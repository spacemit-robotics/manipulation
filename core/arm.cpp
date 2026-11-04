// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Arm state validation, device lease and evidence-based point execution.
#include "arm_session.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <stdexcept>
#include <utility>

namespace spacemit::manipulation {
namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
std::atomic<std::uint64_t> session_counter{0};
constexpr std::size_t kMaximumPathWaypoints = 128;  // Keeps a request inside the worker frame limit.
// Streaming drivers send at most 0.4 deg per 20 ms step: about 20 deg/s is the honest ceiling.
constexpr double kMaximumPathVelocity = 0.35;     // rad/s
constexpr double kMaximumPathAcceleration = 1.0;  // rad/s^2
constexpr double kDegree = 3.14159265358979323846 / 180;
constexpr double kMaximumPathTolerance = 0.8 * kDegree;
// Timed trajectories: bounded size and duration, start on the measured position.
constexpr std::size_t kMaximumTrajectoryKnots = 4000;
constexpr double kMaximumTrajectoryDuration = 60;    // s
constexpr double kMaximumTrajectoryStep = 3 * kDegree;
constexpr double kTrajectoryStartTolerance = 0.5 * kDegree;
constexpr double kMinimumResendInterval = 0.02;  // s
constexpr double kMaximumResendInterval = 0.2;   // s
}  // namespace

void PrepareLockDirectory(const std::string& directory) {
    Check(std::filesystem::path(directory).is_absolute(), "lock directory must be absolute");
    if (mkdir(directory.c_str(), 01777) == 0) {
        // mkdir applies the umask; every account must be able to create its own lease file.
        Check(chmod(directory.c_str(), 01777) == 0, "cannot set shared lock directory mode");
    } else if (errno != EEXIST) {
        throw std::runtime_error("cannot create lock directory " + directory +
            "; set MANIPULATION_LOCK_DIRECTORY at build time");
    }
    struct stat info {};
    Check(lstat(directory.c_str(), &info) == 0 && S_ISDIR(info.st_mode), "lock directory is not a directory");
    // Without the sticky bit another account could unlink a held lease and become a second owner.
    const bool sticky = info.st_mode & S_ISVTX;
    Check(sticky || !(info.st_mode & (S_IWGRP | S_IWOTH)), "shared lock directory must be sticky");
    Check(sticky || info.st_uid == geteuid() || info.st_uid == 0, "lock directory owned by another account");
}

struct ArmSession::Impl {
    DeviceConfig config;
    std::unique_ptr<Arm> arm;
    std::string directory;
    std::string id;
    int lease = -1;
    bool connected = false;
    CommandResult result;
    JointPoint point;
    Clock::time_point deadline{};
    Clock::time_point submitted_at{};
    std::uint64_t last_sample = 0;
    unsigned settled_samples = 0;
    double tolerance = 0;  // Arrival tolerance of the running command.
    Completion completion = Completion::kArrival;
    bool settling = false;
};

void Arm::Enable() { throw CommandRejected("enable unsupported"); }
void Arm::Disable() { throw CommandRejected("disable unsupported"); }
ArmState Arm::Diagnose() { return Read(); }
void Arm::SubmitPath(const JointPath&) { throw CommandRejected("joint path unsupported"); }
void Arm::SubmitTrajectory(const JointTrajectory&) { throw CommandRejected("joint trajectory unsupported"); }

double JointPathDuration(const std::vector<double>& start_rad, const JointPath& path) {
    // Quintic time scaling over the polyline length (largest joint change per segment):
    // peak normalised velocity 1.875 and acceleration 5.774, at least 0.3 s.
    double distance = 0;
    const std::vector<double>* previous = &start_rad;
    for (const auto& waypoint : path.waypoints_rad) {
        double step = 0;
        for (std::size_t i = 0; i < waypoint.size() && i < previous->size(); ++i) {
            step = std::max(step, std::abs(waypoint[i] - (*previous)[i]));
        }
        distance += step;
        previous = &waypoint;
    }
    return std::max({0.3, 1.875 * distance / path.max_velocity_rad_s,
        std::sqrt(5.774 * distance / path.max_acceleration_rad_s2)});
}

ArmSession::ArmSession(DeviceConfig config, std::unique_ptr<Arm> arm, std::string lock_directory)
    : impl_(std::make_unique<Impl>()) {
    ValidateDevice(config);
    Check(arm != nullptr, "arm required");
    Check(std::filesystem::path(lock_directory).is_absolute(), "lock directory must be absolute");
    impl_->config = std::move(config);
    impl_->arm = std::move(arm);
    impl_->directory = std::move(lock_directory);
}

ArmSession::~ArmSession() { Disconnect(); }

void ArmSession::Connect() {
    auto& p = *impl_;
    Check(!p.connected, "already connected");
    PrepareLockDirectory(p.directory);
    const auto path = std::filesystem::path(p.directory) / (p.config.resource_id + ".lock");
    p.lease = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    Check(p.lease >= 0, "cannot open lease; create trusted lock directory first");
    struct stat info {};
    if (fstat(p.lease, &info) != 0 || !S_ISREG(info.st_mode) ||
        flock(p.lease, LOCK_EX | LOCK_NB) != 0) {
        close(p.lease);
        p.lease = -1;
        throw std::runtime_error("device resource busy or invalid lease");
    }
    try {
        p.arm->Connect();
        p.connected = true;
        Read();
        p.result = {};
        p.id = std::to_string(getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count()) +
            "-" + std::to_string(++session_counter);
    } catch (...) {
        p.arm->Disconnect();
        p.connected = false;
        close(p.lease);
        p.lease = -1;
        throw;
    }
}

void ArmSession::Disconnect() noexcept {
    auto& p = *impl_;
    if (p.connected) {
        if (p.result.status == Result::kRunning) {
            try { Stop(Result::kCancelled, "disconnect"); } catch (...) {}
        }
        p.arm->Disconnect();
        p.connected = false;
    }
    p.settling = false;
    if (p.lease >= 0) { close(p.lease); p.lease = -1; }
}

ArmState ArmSession::Read() {
    auto& p = *impl_;
    Check(p.connected, "not connected");
    auto state = p.arm->Read();
    const double age = std::chrono::duration<double>(Clock::now() - state.received_at).count();
    Check(state.valid && state.sample_id > 0 && age >= 0 && age <= p.config.maximum_age_s,
        "incomplete or stale arm feedback");
    Check(state.joints_rad.size() == p.config.joints.size(), "arm feedback joint count mismatch");
    for (double q : state.joints_rad) Check(std::isfinite(q), "nonfinite arm feedback");
    if (!state.moving && p.result.status != Result::kRunning) p.settling = false;  // Seen still again.
    return state;
}

void ArmSession::Ready() {
    const auto status = impl_->result.status;
    Check(status != Result::kRunning, "arm busy");
    Check(status != Result::kFault && status != Result::kTimedOut && status != Result::kStopUnknown,
        "fault latched; inspect before reconnecting");
}

ArmState ArmSession::Diagnose() {
    Ready();
    Read();
    impl_->arm->Diagnose();
    return Read();
}

ArmCapabilities ArmSession::GetCapabilities() const { return impl_->arm->GetCapabilities(); }

void ArmSession::Enable() {
    Ready();
    if (!GetCapabilities().enable) throw CommandRejected("enable unsupported");
    const auto state = Read();
    if (state.fault || state.moving || state.command_active) {
        throw CommandRejected("cannot enable faulted or moving arm");
    }
    try {
        impl_->arm->Enable();
        Check(Read().enabled, "enable not confirmed");
    } catch (const CommandRejected&) { throw; }
    catch (...) {
        impl_->result.status = Result::kFault;
        impl_->result.detail = "enable failed or unconfirmed; inspect before reconnecting";
        throw;
    }
}

void ArmSession::Disable() {
    Ready();
    if (!GetCapabilities().disable) throw CommandRejected("disable unsupported");
    const auto state = Read();
    if (state.fault || state.moving || state.command_active) {
        throw CommandRejected("cannot disable faulted or moving arm");
    }
    try {
        impl_->arm->Disable();
        Check(!Read().enabled, "disable not confirmed");
    } catch (const CommandRejected&) { throw; }
    catch (...) {
        impl_->result.status = Result::kFault;
        impl_->result.detail = "disable failed or unconfirmed; inspect before reconnecting";
        throw;
    }
}

CommandResult ArmSession::Hold() {
    auto& p = *impl_;
    Check(p.connected, "not connected");
    if (!GetCapabilities().verified_hold) throw CommandRejected("verified arm hold unsupported");
    const auto state = Read();
    if (state.fault) {
        p.result.status = Result::kFault;
        p.result.detail = "arm fault observed before hold";
    }
    if (!state.enabled) throw CommandRejected("cannot hold a disabled arm");
    bool confirmed = false;
    try { confirmed = p.arm->Hold(); }
    catch (const CommandRejected&) { throw; }
    catch (...) {}  // A failed hold cannot establish a physical stop.
    CommandResult result{p.result.command_id + 1,
        confirmed ? Result::kSucceeded : Result::kStopUnknown,
        confirmed ? "arm_hold_confirmed" : "arm hold unconfirmed; inspect before reconnecting"};
    const bool latched = p.result.status == Result::kFault || p.result.status == Result::kTimedOut ||
        p.result.status == Result::kStopUnknown;
    if (!confirmed || !latched) p.result = result;
    if (confirmed) p.settling = false;
    return result;
}

void ArmSession::CheckTarget(const std::vector<double>& target_rad, double speed_scale) {
    auto& p = *impl_;
    const auto capabilities = p.arm->GetCapabilities();
    Check(target_rad.size() == p.config.joints.size(), "target joint count mismatch");
    Check(std::isfinite(speed_scale) && speed_scale > 0 && speed_scale <= 1, "speed_scale must be in (0, 1]");
    Check(speed_scale >= capabilities.minimum_speed_scale, "speed_scale below driver minimum");
    for (std::size_t i = 0; i < target_rad.size(); ++i) {
        Check(std::isfinite(target_rad[i]) && target_rad[i] >= p.config.joints[i].minimum_rad &&
            target_rad[i] <= p.config.joints[i].maximum_rad, "target outside joint limits");
    }
}

template <typename Extra, typename Write>
CommandResult ArmSession::Begin(const JointPoint& final_point, double tolerance_rad, Completion completion,
    Extra extra_time_s, Write write) {
    auto& p = *impl_;
    auto state = Read();
    // Motion is allowed only when it is the arm settling after our own command that did not wait.
    Check(!state.fault && state.enabled && state.position_mode && (!state.moving || p.settling),
        "arm must be normal, enabled, stationary and in position mode");
    p.point = final_point;
    p.tolerance = tolerance_rad;
    p.completion = completion;
    p.settling = false;
    p.last_sample = state.sample_id;
    p.settled_samples = 0;
    p.submitted_at = Clock::now();
    p.deadline = p.submitted_at + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(p.config.timeout_s + extra_time_s(state)));
    const auto previous = p.result;
    p.result = {p.result.command_id + 1, Result::kRunning, "accepted; awaiting fresh target evidence"};
    try { write(); }
    catch (const CommandRejected&) { p.result = previous; throw; }  // Nothing written; no stop needed.
    catch (const std::exception& error) { Stop(Result::kFault, error.what()); }
    catch (...) { Stop(Result::kFault, "arm submission failed"); }
    return p.result;
}

namespace {
double ResolveTolerance(double requested, double device) {
    Check(std::isfinite(requested) && requested >= 0 && requested <= kMaximumPathTolerance,
        "tolerance must be in [0, 0.8 deg]");
    return requested == 0 ? device : requested;
}
}  // namespace

CommandResult ArmSession::Submit(const JointPoint& point) {
    Ready();
    Check(impl_->arm->GetCapabilities().joint_point, "joint point unsupported");
    CheckTarget(point.target_rad, point.speed_scale);
    Check(std::isfinite(point.resend_interval_s) && (point.resend_interval_s == 0 ||
        (point.resend_interval_s >= kMinimumResendInterval && point.resend_interval_s <= kMaximumResendInterval)),
        "resend_interval_s must be 0 or in [0.02, 0.2] s");
    auto resolved = point;  // The driver receives the tolerance arrival is judged with.
    resolved.tolerance_rad = ResolveTolerance(point.tolerance_rad, impl_->config.tolerance_rad);
    return Begin(resolved, resolved.tolerance_rad,
        point.wait_for_arrival ? Completion::kArrival : Completion::kWithinTolerance,
        [](const ArmState&) { return 0.0; }, [&] { impl_->arm->Submit(resolved); });
}

CommandResult ArmSession::Follow(const JointPoint& point) {
    Ready();
    Check(impl_->arm->GetCapabilities().joint_point, "joint point unsupported");
    CheckTarget(point.target_rad, point.speed_scale);
    Check(point.resend_interval_s == 0, "a followed target is replaced, never re-sent");
    auto resolved = point;
    resolved.tolerance_rad = ResolveTolerance(point.tolerance_rad, impl_->config.tolerance_rad);
    Begin(resolved, resolved.tolerance_rad, Completion::kDelivered, [](const ArmState&) { return 0.0; },
        [&] { impl_->arm->Submit(resolved); });
    auto& p = *impl_;
    if (p.result.status == Result::kRunning) {
        p.result.status = Result::kSucceeded;
        p.result.detail = "sent: target sent; arrival not awaited";
        p.settling = true;
    }
    return p.result;
}

CommandResult ArmSession::SubmitTrajectory(const JointTrajectory& trajectory) {
    Ready();
    Check(impl_->arm->GetCapabilities().joint_trajectory, "joint trajectory unsupported");
    const auto& times = trajectory.times_s;
    const auto& knots = trajectory.positions_rad;
    Check(knots.size() >= 2 && knots.size() <= kMaximumTrajectoryKnots && times.size() == knots.size(),
        "trajectory needs 2..4000 knots with one time each");
    Check(std::isfinite(trajectory.command_rate_hz) && trajectory.command_rate_hz >= 20 &&
        trajectory.command_rate_hz <= 500, "command_rate_hz must be in [20, 500]");
    Check(std::isfinite(trajectory.max_step_rad) && trajectory.max_step_rad > 0 &&
        trajectory.max_step_rad <= kMaximumTrajectoryStep * (1 + 1e-9), "max_step_rad must be in (0, 3 deg]");
    Check(times.front() == 0 && std::isfinite(times.back()) && times.back() <= kMaximumTrajectoryDuration,
        "trajectory times must start at 0 and last at most 60 s");
    const double tick = 1 / trajectory.command_rate_hz;
    for (std::size_t i = 0; i < knots.size(); ++i) {
        CheckTarget(knots[i], trajectory.speed_scale);
        if (i == 0) continue;
        Check(std::isfinite(times[i]) && times[i] > times[i - 1], "trajectory times must increase");
        // A linear segment sent at command_rate_hz never steps further than max_step_rad per tick.
        const double ticks = std::max(1.0, (times[i] - times[i - 1]) / tick);
        for (std::size_t j = 0; j < knots[i].size(); ++j) {
            Check(std::abs(knots[i][j] - knots[i - 1][j]) <= trajectory.max_step_rad * ticks + 1e-9,
                "trajectory step exceeds max_step_rad");
        }
    }
    auto resolved = trajectory;
    resolved.tolerance_rad = ResolveTolerance(trajectory.tolerance_rad, impl_->config.tolerance_rad);
    return Begin({knots.back(), trajectory.speed_scale}, resolved.tolerance_rad,
        trajectory.wait_for_arrival ? Completion::kArrival : Completion::kDelivered,
        [&](const ArmState& state) {
            for (std::size_t j = 0; j < knots.front().size(); ++j) {
                Check(std::abs(state.joints_rad[j] - knots.front()[j]) <= kTrajectoryStartTolerance,
                    "trajectory must start at the current position (0.5 deg)");
            }
            return times.back();
        },
        [&] { impl_->arm->SubmitTrajectory(resolved); });
}

CommandResult ArmSession::SubmitPath(const JointPath& path) {
    Ready();
    Check(impl_->arm->GetCapabilities().joint_path, "joint path unsupported");
    Check(!path.waypoints_rad.empty() && path.waypoints_rad.size() <= kMaximumPathWaypoints,
        "joint path needs 1..128 waypoints");
    Check(std::isfinite(path.max_velocity_rad_s) && path.max_velocity_rad_s > 0 &&
        path.max_velocity_rad_s <= kMaximumPathVelocity, "path velocity must be in (0, 0.35] rad/s");
    Check(std::isfinite(path.max_acceleration_rad_s2) && path.max_acceleration_rad_s2 > 0 &&
        path.max_acceleration_rad_s2 <= kMaximumPathAcceleration, "path acceleration must be in (0, 1] rad/s^2");
    for (const auto& waypoint : path.waypoints_rad) CheckTarget(waypoint, path.speed_scale);
    auto resolved = path;  // The driver receives the tolerance arrival is judged with.
    resolved.tolerance_rad = ResolveTolerance(path.tolerance_rad, impl_->config.tolerance_rad);
    // The deadline covers the planned profile from the current position plus the configured timeout.
    return Begin({path.waypoints_rad.back(), path.speed_scale}, resolved.tolerance_rad, Completion::kArrival,
        [&](const ArmState& state) { return JointPathDuration(state.joints_rad, path); },
        [&] { impl_->arm->SubmitPath(resolved); });
}

void ArmSession::Stop(Result result, const std::string& detail) {
    auto& p = *impl_;
    p.result.status = Result::kStopUnknown;  // Preserve uncertainty even when Hold throws.
    p.result.detail = detail;
    try {
        if (p.arm->GetCapabilities().verified_hold && p.arm->Hold()) p.result.status = result;
    } catch (...) {}
}

CommandResult ArmSession::Poll() {
    auto& p = *impl_;
    if (p.result.status != Result::kRunning) return p.result;
    try {
        if (Clock::now() >= p.deadline) { Stop(Result::kTimedOut, "point deadline exceeded"); return p.result; }
        const auto state = Read();
        Check(!state.fault && state.enabled && state.position_mode, "arm mode/enable/fault changed");
        if (state.sample_id < p.last_sample) throw std::runtime_error("feedback sequence regressed");
        if (state.sample_id == p.last_sample || state.received_at <= p.submitted_at) return p.result;
        p.last_sample = state.sample_id;
        if (p.completion == Completion::kDelivered) {
            if (!state.command_active) {
                p.result.status = Result::kSucceeded;
                p.result.detail = "delivered: last command sent; arrival not awaited";
                p.settling = true;
            }
            return p.result;
        }
        bool within = !state.command_active;
        for (std::size_t i = 0; i < p.point.target_rad.size(); ++i) {
            within = within && std::abs(state.joints_rad[i] - p.point.target_rad[i]) <= p.tolerance;
        }
        if (p.completion == Completion::kWithinTolerance) {
            p.settled_samples = within ? p.settled_samples + 1 : 0;
            if (p.settled_samples >= 2) {
                p.result.status = Result::kSucceeded;
                p.result.detail = "within_tolerance: two distinct fresh samples; stillness not awaited";
                p.settling = true;
            }
            return p.result;
        }
        const bool reached = within && !state.moving;
        p.settled_samples = reached ? p.settled_samples + 1 : 0;
        if (p.settled_samples >= 3) {
            p.result.status = Result::kSucceeded;
            p.result.detail = "target_reached: three distinct fresh stationary samples";
        }
    } catch (const std::exception& error) { Stop(Result::kFault, error.what()); }
    catch (...) { Stop(Result::kFault, "arm feedback failed"); }
    return p.result;
}

CommandResult ArmSession::Cancel() {
    if (impl_->result.status == Result::kRunning) Stop(Result::kCancelled, "cancel requested");
    return impl_->result;
}
CommandResult ArmSession::GetResult() const { return impl_->result; }
bool ArmSession::Settling() const { return impl_->settling; }
const std::vector<double>& ArmSession::LastTarget() const { return impl_->point.target_rad; }
const std::string& ArmSession::SessionId() const { return impl_->id; }
}  // namespace spacemit::manipulation
