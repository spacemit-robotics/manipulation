// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Assembly ownership, normalized effector feedback and evidence-based command completion.
#include "manipulation/device.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include "arm_session.h"

namespace spacemit::manipulation {
namespace {
void Check(bool condition, const char* detail) {
    if (!condition) throw std::runtime_error(detail);
}
bool Failed(Result status) {
    return status == Result::kFault || status == Result::kTimedOut || status == Result::kStopUnknown;
}
constexpr double kToolArmDrift = 3.14159265358979323846 / 180;  // 1 degree
}  // namespace
void EndEffector::Enable() { throw CommandRejected("effector enable unsupported"); }
void EndEffector::Disable() { throw CommandRejected("effector disable unsupported"); }
void ValidateEffector(const EffectorConfig& config) {
    if (config.channels.empty()) return;
    Check(!config.device_id.empty() && config.channels.size() <= 32, "invalid effector identity/channels");
    std::set<std::string> names;
    for (const auto& channel : config.channels) {
        Check(!channel.empty() && channel.size() <= 80 && names.insert(channel).second, "invalid channel name");
    }
    Check(std::isfinite(config.tolerance) && config.tolerance > 0 && config.tolerance <= 0.1,
        "invalid effector tolerance");
    Check(std::isfinite(config.timeout_s) && config.timeout_s > 0 && config.timeout_s <= 120,
        "invalid effector timeout");
    Check(std::isfinite(config.maximum_age_s) && config.maximum_age_s > 0 && config.maximum_age_s <= 1,
        "invalid effector feedback age");
}
void DeviceRegistry::Register(DeviceDriver driver) {
    Check(!driver.id.empty() && driver.interface_version == 1 && driver.validate && driver.create,
        "invalid device driver registration");
    Check(std::isfinite(driver.minimum_arm_speed_scale) && driver.minimum_arm_speed_scale >= 0 &&
        driver.minimum_arm_speed_scale <= 1 && std::isfinite(driver.maximum_effector_duration_s) &&
        driver.maximum_effector_duration_s > 0, "invalid device capability metadata");
    const auto id = driver.id;
    Check(drivers_.emplace(id, std::move(driver)).second, "duplicate device driver");
}
const DeviceDriver& DeviceRegistry::Get(const std::string& id) const {
    const auto found = drivers_.find(id);
    Check(found != drivers_.end(), "unknown device driver");
    return found->second;
}
std::vector<std::string> DeviceRegistry::List() const {
    std::vector<std::string> ids;
    for (const auto& entry : drivers_) ids.push_back(entry.first);
    return ids;
}
void DeviceRegistry::Validate(const std::string& id, const DeviceDefinition& definition,
    const DriverOptions& options) const {
    ValidateDevice(definition.arm);
    ValidateEffector(definition.effector);
    const auto found = drivers_.find(id);
    Check(found != drivers_.end(), "unknown device driver");
    found->second.validate(definition, options);
}
Devices DeviceRegistry::Create(const std::string& id, const DeviceDefinition& definition,
    const DriverOptions& options) const {
    Validate(id, definition, options);
    auto devices = drivers_.at(id).create(definition, options);
    Check(devices.arm != nullptr && static_cast<bool>(devices.effector) == !definition.effector.channels.empty(),
        "driver device composition mismatch");
    return devices;
}
struct DeviceSession::Impl {
    DeviceDefinition definition;
    std::unique_ptr<ArmSession> arm;
    std::unique_ptr<EndEffector> effector;
    bool connected = false;
    bool latched = false;
    std::uint64_t command_id = 0;
};
DeviceSession::DeviceSession(DeviceDefinition definition, Devices devices)
    : DeviceSession(std::move(definition), std::move(devices), DefaultLockDirectory()) {}
DeviceSession::DeviceSession(DeviceDefinition definition, Devices devices, std::string lock_directory)
    : impl_(std::make_unique<Impl>()) {
    ValidateEffector(definition.effector);
    Check(static_cast<bool>(devices.effector) == !definition.effector.channels.empty(), "effector required");
    impl_->arm = std::make_unique<ArmSession>(definition.arm, std::move(devices.arm), std::move(lock_directory));
    impl_->effector = std::move(devices.effector);
    impl_->definition = std::move(definition);
}
DeviceSession::~DeviceSession() { Disconnect(); }
void DeviceSession::Connect() {
    auto& p = *impl_;
    Check(!p.connected, "already connected");
    p.arm->Connect();  // Acquire the whole assembly lease before any effector connection.
    try {
        if (p.effector) p.effector->Connect();
        p.connected = true;
        p.latched = false;
        ReadArm();
        if (p.effector) ReadEffector();
    } catch (...) { Disconnect(); throw; }
}
void DeviceSession::Disconnect() noexcept {
    if (impl_->effector) impl_->effector->Disconnect();
    impl_->arm->Disconnect();
    impl_->connected = false;
}
void DeviceSession::Ready() const {
    Check(impl_->connected && !impl_->latched, "device disconnected or fault latched; inspect before reconnect");
}
ArmState DeviceSession::ReadArm() {
    Check(impl_->connected, "not connected");
    const auto state = impl_->arm->Read();
    impl_->latched = impl_->latched || state.fault;
    return state;
}
EffectorState DeviceSession::ReadEffector() {
    auto& p = *impl_;
    Check(p.connected && p.effector, "effector not connected");
    const auto state = p.effector->Read();
    const auto& config = p.definition.effector;
    const double age = std::chrono::duration<double>(Clock::now() - state.received_at).count();
    Check(state.valid && state.sample_id > 0 && age >= 0 && age <= config.maximum_age_s,
        "invalid or stale effector feedback");
    if (config.feedback == EffectorFeedback::kNone) {
        // Liveness only: a fresh sample proves the device is on the bus, nothing about its position.
        Check(state.positions.empty(), "effector configured without feedback reported positions");
        p.latched = p.latched || state.fault;
        return state;
    }
    Check(state.positions.size() == config.channels.size(), "effector feedback channel count mismatch");
    for (double value : state.positions) {
        Check(std::isfinite(value) && value >= 0 && value <= 1, "invalid normalized effector feedback");
    }
    Check(state.loads.empty() || state.loads.size() == config.channels.size(), "effector load channel count mismatch");
    for (double value : state.loads) {
        Check(std::isfinite(value) && value >= -1 && value <= 1, "invalid normalized effector load");
    }
    Check(!state.holding.has_value() || p.effector->GetCapabilities().grasp_detection,
        "effector reported holding without grasp detection");
    p.latched = p.latched || state.fault;
    return state;
}
void DeviceSession::Diagnose() {
    Check(impl_->connected, "not connected");
    const auto arm = impl_->arm->Diagnose();
    impl_->latched = impl_->latched || arm.fault;
    if (HasEffector()) ReadEffector();
}
void DeviceSession::EnableArm() {
    Ready();
    try { impl_->arm->Enable(); }
    catch (const CommandRejected&) { throw; }
    catch (...) { impl_->latched = true; throw; }
}
void DeviceSession::DisableArm() {
    Ready();
    try { impl_->arm->Disable(); }
    catch (const CommandRejected&) { throw; }
    catch (...) { impl_->latched = true; throw; }
}
ArmCapabilities DeviceSession::GetArmCapabilities() const { return impl_->arm->GetCapabilities(); }
EffectorCapabilities DeviceSession::GetEffectorCapabilities() const {
    if (!HasEffector()) throw CommandRejected("no end effector configured");
    return impl_->effector->GetCapabilities();
}
void DeviceSession::SetEffectorEnabled(bool enabled) {
    Ready();
    const auto capabilities = GetEffectorCapabilities();
    if (!(enabled ? capabilities.enable : capabilities.disable)) {
        throw CommandRejected(enabled ? "effector enable unsupported" : "effector disable unsupported");
    }
    const auto arm = ReadArm();
    const auto tool = ReadEffector();
    if (arm.fault || arm.moving || arm.command_active || tool.fault || tool.moving) {
        throw CommandRejected("cannot change torque while arm or effector is faulted or moving");
    }
    try {
        if (enabled) impl_->effector->Enable();
        else impl_->effector->Disable();
        const auto state = ReadEffector();
        Check(state.enabled.has_value() && *state.enabled == enabled, "effector torque state not confirmed");
    } catch (const CommandRejected&) { throw; }
    catch (...) { impl_->latched = true; throw; }
}
void DeviceSession::EnableEffector() { SetEffectorEnabled(true); }
void DeviceSession::DisableEffector() { SetEffectorEnabled(false); }
CommandResult DeviceSession::HoldArm() {
    Check(impl_->connected, "not connected");
    CommandResult result;
    try { result = impl_->arm->Hold(); }
    catch (...) {
        impl_->latched = impl_->latched || Failed(impl_->arm->GetResult().status);
        throw;
    }
    impl_->latched = impl_->latched || Failed(result.status) || Failed(impl_->arm->GetResult().status);
    result.command_id = ++impl_->command_id;
    return result;
}
CommandResult DeviceSession::HoldEffector() {
    auto& p = *impl_;
    Check(p.connected, "not connected");
    if (!GetEffectorCapabilities().verified_hold || p.definition.effector.feedback == EffectorFeedback::kNone) {
        throw CommandRejected("verified effector hold unsupported");
    }
    const auto state = ReadEffector();
    if (state.enabled.has_value() && !*state.enabled) throw CommandRejected("cannot hold a disabled effector");
    bool confirmed = false;
    try { confirmed = p.effector->Hold(); }
    catch (const CommandRejected&) { throw; }
    catch (...) {}
    p.latched = p.latched || !confirmed;
    return {++p.command_id, confirmed ? Result::kSucceeded : Result::kStopUnknown,
        confirmed ? "effector_hold_confirmed" : "effector hold unconfirmed; inspect before reconnecting"};
}
bool DeviceSession::HasEffector() const { return impl_->effector != nullptr; }
bool DeviceSession::Latched() const { return impl_->latched; }
const DeviceDefinition& DeviceSession::Definition() const { return impl_->definition; }
CommandResult DeviceSession::MoveArm(const JointPoint& point, const std::function<bool()>& cancelled) {
    return RunArm([&] { return impl_->arm->Submit(point); }, cancelled);
}
CommandResult DeviceSession::FollowArm(const JointPoint& point) {
    Ready();
    auto& p = *impl_;
    if (p.effector) Check(!ReadEffector().fault, "effector faulted before arm motion");
    auto result = p.arm->Follow(point);
    p.latched = Failed(result.status);
    result.command_id = ++p.command_id;
    return result;
}
CommandResult DeviceSession::FollowEffector(const EffectorPoint& point) {
    Ready();
    auto& p = *impl_;
    CheckEffectorPoint(point);
    const auto arm = ReadArm();
    Check(!arm.fault && arm.enabled && arm.position_mode, "arm not ready for tool motion");
    const auto before = ReadEffector();
    Check(!before.fault, "effector faulted");
    Check(!before.enabled.has_value() || *before.enabled, "effector disabled");
    const bool open_loop = p.definition.effector.feedback == EffectorFeedback::kNone;
    CommandResult result{++p.command_id, open_loop ? Result::kUnverified : Result::kSucceeded,
        "sent: target sent; arrival not awaited"};
    try {
        p.effector->Submit(point);
        return result;
    } catch (const CommandRejected&) {
        throw;  // Refused before any device write: nothing to stop or latch.
    } catch (const std::exception& error) {
        result.detail = error.what();
    } catch (...) {
        result.detail = "unknown effector failure";
    }
    result.status = Result::kFault;
    try {
        if (open_loop || !p.effector->Hold()) result.status = Result::kStopUnknown;
    } catch (...) { result.status = Result::kStopUnknown; }
    p.latched = true;
    return result;
}
CommandResult DeviceSession::JogArm(std::size_t joint_index, double delta_rad, double speed_scale,
    const std::function<bool()>& cancelled) {
    Ready();
    if (joint_index >= impl_->definition.arm.joints.size() || !std::isfinite(delta_rad)) {
        throw CommandRejected("invalid joint index or nonfinite step");
    }
    auto target = ReadArm().joints_rad;
    target[joint_index] += delta_rad;
    return MoveArm({target, speed_scale}, cancelled);
}
CommandResult DeviceSession::MoveArmPath(const JointPath& path, const std::function<bool()>& cancelled) {
    return RunArm([&] { return impl_->arm->SubmitPath(path); }, cancelled);
}
CommandResult DeviceSession::MoveArmTrajectory(const JointTrajectory& trajectory, const std::function<bool()>& cancelled,
    const std::vector<TimedEffectorPoint>& effector) {
    return RunArm([&] { return impl_->arm->SubmitTrajectory(trajectory); }, cancelled, effector);
}
void DeviceSession::CheckEffectorPoint(const EffectorPoint& point) const {
    const auto& p = *impl_;
    Check(p.effector && point.positions.size() == p.definition.effector.channels.size(), "invalid tool target");
    Check(std::isfinite(point.duration_s) && point.duration_s > 0 &&
        point.duration_s <= p.effector->GetCapabilities().maximum_duration_s,
        "effector duration unsupported");
    for (double value : point.positions) {
        Check(std::isfinite(value) && value >= 0 && value <= 1, "effector target outside [0,1]");
    }
}
CommandResult DeviceSession::RunArm(const std::function<CommandResult()>& submit,
    const std::function<bool()>& cancelled, const std::vector<TimedEffectorPoint>& effector) {
    Ready();
    auto& p = *impl_;
    Check(effector.size() <= 64, "at most 64 timed effector targets");
    for (std::size_t i = 0; i < effector.size(); ++i) {
        CheckEffectorPoint(effector[i].point);
        Check(std::isfinite(effector[i].at_s) && effector[i].at_s >= (i ? effector[i - 1].at_s : 0) &&
            effector[i].at_s <= 60, "effector targets need ordered times within 60 s");
    }
    if (cancelled && cancelled()) return {++p.command_id, Result::kCancelled, "cancelled before submission"};
    if (p.effector) {
        const auto tool = ReadEffector();
        Check(!tool.fault && !tool.moving, "effector faulted or busy before arm motion");
        Check(effector.empty() || !tool.enabled.has_value() || *tool.enabled,
            "effector disabled before scheduled targets");
    }
    auto result = submit();
    const auto started = Clock::now();
    const auto after = [&](double seconds) {
        return started + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    };
    std::size_t sent = 0;
    auto settled = started;  // When the last sent effector target has had its duration.
    try {
        while (true) {
            const bool cancel = cancelled && cancelled();
            if (result.status == Result::kRunning) result = cancel ? p.arm->Cancel() : p.arm->Poll();
            const bool arm_ok = result.status == Result::kRunning || result.status == Result::kSucceeded;
            if (arm_ok && !cancel && sent < effector.size() && Clock::now() >= after(effector[sent].at_s)) {
                const auto tool = ReadEffector();
                Check(!tool.fault && (!tool.enabled.has_value() || *tool.enabled),
                    "effector fault or disabled during timed targets");
                p.effector->Submit(effector[sent].point);  // One per cycle: late targets are never burst.
                settled = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(effector[sent].point.duration_s));
                ++sent;
            }
            if (!arm_ok) break;
            if (result.status == Result::kSucceeded && (sent == effector.size() && Clock::now() >= settled)) break;
            if (cancel && result.status == Result::kSucceeded) {
                result = {result.command_id, Result::kCancelled, "cancelled during timed effector targets"};
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (...) {
        result = p.arm->Cancel();
        if (result.status != Result::kStopUnknown) result.status = Result::kFault;
        result.detail = "callback/command processing failed; hold attempted, no replay";
    }
    if (sent && result.status != Result::kSucceeded) {
        // A sent effector target cannot be proven stopped without a verified hold.
        try {
            if (p.definition.effector.feedback == EffectorFeedback::kNone || !p.effector->Hold()) {
                result.status = Result::kStopUnknown;
            }
        } catch (...) { result.status = Result::kStopUnknown; }
    } else if (sent) {
        result.status = Result::kUnverified;
        result.detail += "; " + std::to_string(sent) + " effector targets delivered on schedule, not measured";
    }
    p.latched = Failed(result.status);
    result.command_id = ++p.command_id;
    return result;
}
CommandResult DeviceSession::MoveEffector(const EffectorPoint& point, const std::function<bool()>& cancelled) {
    Ready();
    auto& p = *impl_;
    CheckEffectorPoint(point);
    if (cancelled && cancelled()) return {++p.command_id, Result::kCancelled, "cancelled before submission"};
    const auto arm = ReadArm();
    // The arm may still be settling after a command that did not wait for arrival. It may keep
    // converging to that command's target during the tool motion, as the original executor allowed;
    // any other motion of more than kToolArmDrift (the original hand start tolerance) is a fault:
    // a still arm must stay where it was, a settling one must not get farther from its target.
    Check(!arm.fault && arm.enabled && arm.position_mode && (!arm.moving || p.arm->Settling()),
        "arm not ready for tool motion");
    const auto reference = p.arm->Settling() ? p.arm->LastTarget() : arm.joints_rad;
    const auto distance = [&reference](const std::vector<double>& joints) {
        double value = 0;
        for (std::size_t i = 0; i < joints.size() && i < reference.size(); ++i) {
            value = std::max(value, std::abs(joints[i] - reference[i]));
        }
        return value;
    };
    const double allowed = distance(arm.joints_rad) + kToolArmDrift;
    const auto before = ReadEffector();
    Check(!before.fault && !before.moving, "effector faulted or busy");
    Check(!before.enabled.has_value() || *before.enabled, "effector disabled");
    const bool open_loop = p.definition.effector.feedback == EffectorFeedback::kNone;
    Check(!open_loop || point.duration_s <= p.definition.effector.timeout_s,
        "effector duration exceeds timeout_s");
    const auto submitted = Clock::now();
    // Open loop: wait the commanded execution time, then report the move as delivered, not reached.
    const auto finished = submitted + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(point.duration_s));
    const auto deadline = submitted + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(p.definition.effector.timeout_s));
    CommandResult result{++p.command_id, Result::kRunning, "awaiting fresh effector feedback"};
    std::uint64_t last = before.sample_id;
    unsigned settled = 0;
    unsigned gripping = 0;  // Fresh samples stopped on an object (drivers with grasp detection).
    bool written = false;
    try {
        p.effector->Submit(point);
        written = true;
        while (true) {
            if (cancelled && cancelled()) { result.status = Result::kCancelled; break; }
            if (open_loop && Clock::now() >= finished) {  // Health was checked at most one cycle ago.
                return {result.command_id, Result::kUnverified,
                    "command delivered, execution time elapsed; position not measured"};
            }
            if (Clock::now() >= deadline) { result.status = Result::kTimedOut; break; }
            const auto current_arm = ReadArm();
            const bool moved = distance(current_arm.joints_rad) > allowed;
            if (current_arm.fault || !current_arm.enabled || !current_arm.position_mode || moved) {
                throw std::runtime_error(std::string("arm state changed during effector motion:") +
                    (current_arm.fault ? " fault" : "") + (!current_arm.enabled ? " disabled" : "") +
                    (!current_arm.position_mode ? " not_position_mode" : "") +
                    (moved ? " moved" : ""));
            }
            const auto state = ReadEffector();
            Check(!state.fault && state.sample_id >= last, "effector fault/sequence regression");
            if (!open_loop && state.sample_id > last && state.received_at > submitted) {
                last = state.sample_id;
                bool reached = !state.moving;
                for (std::size_t i = 0; i < point.positions.size(); ++i) {
                    reached = reached && std::abs(state.positions[i] - point.positions[i]) <= p.definition.effector.tolerance;
                }
                settled = reached ? settled + 1 : 0;
                gripping = !state.moving && state.holding.value_or(false) ? gripping + 1 : 0;
                if (settled >= 3) return {result.command_id, Result::kSucceeded, "effector_position_reached"};
                // Stopped on an object short of a closing target: a grasp, not a failure to reach.
                if (gripping >= 3) return {result.command_id, Result::kSucceeded, "effector_grasp_holding"};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const CommandRejected& error) {
        if (!written) throw;  // Refused before any device write: nothing to stop or latch.
        result.status = Result::kFault;
        result.detail = error.what();
    } catch (const std::exception& error) { result.status = Result::kFault; result.detail = error.what(); }
    catch (...) { result.status = Result::kFault; result.detail = "unknown effector/callback failure"; }
    try {
        // Without feedback no driver can confirm a stop, whatever Hold() reports.
        if (open_loop || !p.effector->Hold()) result.status = Result::kStopUnknown;
    } catch (...) { result.status = Result::kStopUnknown; }
    p.latched = Failed(result.status);
    if (result.detail == "awaiting fresh effector feedback") result.detail = "effector interrupted; no task replay";
    return result;
}
}  // namespace spacemit::manipulation
