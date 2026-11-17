// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// One real-time physics owner and immutable feedback snapshots for both roles.
#include "mujoco_session.h"

#if defined(__linux__)
#include <sys/resource.h>
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <set>
#include <stdexcept>
#include <utility>

namespace spacemit::manipulation::simulation {
namespace {
struct ThreadUsage {
    double cpu_ms = 0;
    long minor_faults = 0;
    long major_faults = 0;
    bool valid = false;
};

ThreadUsage ReadThreadUsage() noexcept {
    ThreadUsage result;
#if defined(__linux__) && defined(RUSAGE_THREAD)
    rusage usage{};
    if (getrusage(RUSAGE_THREAD, &usage) == 0) {
        result.cpu_ms = 1000.0 * (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
                0.001 * (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec);
        result.minor_faults = usage.ru_minflt;
        result.major_faults = usage.ru_majflt;
        result.valid = true;
    }
#endif
    return result;
}

void Require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

void Reject(bool condition, const char* detail) {
    if (!condition) throw CommandRejected(detail);
}

bool InRange(double value, double minimum, double maximum) {
    return std::isfinite(value) && value >= minimum && value <= maximum;
}

double Normalize(double position, const EffectorMapping& mapping, double tolerance) {
    const auto value =
            (position - mapping.joint_closed) / (mapping.joint_open - mapping.joint_closed);
    // Soft physical joint limits can settle just beyond an endpoint. Public positions have a
    // closed [0,1] domain: saturate only within the configured tolerance, capped at one percent.
    // Larger excursions are faults; velocity remains measured, and no grasp is inferred.
    const double slack = std::min(tolerance, 0.01);
    Require(InRange(value, -slack, 1 + slack), "MuJoCo effector outside mapped position endpoints");
    return std::clamp(value, 0.0, 1.0);
}

double Control(double position, const EffectorMapping& mapping) {
    return mapping.control_closed + position * (mapping.control_open - mapping.control_closed);
}

void CheckControlRange(const mujoco_sim::ActuatorInfo& actuator, double first, double second) {
    Require(!actuator.control_limited ||
                    (InRange(first, actuator.minimum, actuator.maximum) &&
                            InRange(second, actuator.minimum, actuator.maximum)),
            "MuJoCo actuator control range does not cover configured endpoints");
}

void CheckJointRange(const mujoco_sim::JointInfo& joint, double first, double second) {
    Require(!joint.limited ||
                    (InRange(first, joint.minimum, joint.maximum) &&
                            InRange(second, joint.minimum, joint.maximum)),
            "MuJoCo joint range does not cover configured endpoints");
}
}  // namespace

SharedSession::SharedSession(
        Mapping mapping, std::optional<DeviceConfig> arm, std::optional<EffectorConfig> effector) {
    mapping_ = std::move(mapping);
    arm_config_ = std::move(arm);
    effector_config_ = std::move(effector);
    // Factories can be called separately from validation; the mapping file may have changed.
    if (arm_config_) ValidateArmMapping(*arm_config_, mapping_);
    if (effector_config_) ValidateEffectorMapping(*effector_config_, mapping_);
}

SharedSession::~SharedSession() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    Stop();
}

void SharedSession::ValidateModel() {
    Require(InRange(world_->Timestep(), 0.00001, 0.02),
            "MuJoCo timestep must be in [0.00001, 0.02] s");
    if (arm_config_) {
        for (std::size_t index = 0; index < mapping_.arm.size(); ++index) {
            const auto& mapping = mapping_.arm[index];
            const auto joint = world_->Joint(mapping.joint);
            const auto actuator = world_->Actuator(mapping.actuator);
            Require(joint.type == mujoco_sim::JointType::kHinge,
                    "MuJoCo arm contract requires hinge joints in radians");
            Require(actuator.position_control && actuator.joint_transmission && actuator.enabled &&
                            actuator.joint_name == mapping.joint &&
                            std::abs(actuator.gear - 1) <= 1e-12,
                    "MuJoCo arm requires a unit position servo on the mapped joint");
            const auto& limit = arm_config_->joints[index];
            CheckJointRange(joint, limit.minimum_rad, limit.maximum_rad);
            CheckControlRange(actuator, limit.minimum_rad, limit.maximum_rad);
            Require(InRange(joint.position, limit.minimum_rad, limit.maximum_rad),
                    "MuJoCo initial joint position outside configured limits");
            Require(std::abs(actuator.control - joint.position) <= 1e-6 &&
                            std::abs(joint.velocity) <= mapping_.arm_moving_threshold_rad_s,
                    "MuJoCo initial arm control/velocity would start an unrequested motion");
        }
    }
    if (effector_config_) {
        std::set<std::string> controlled_joints;
        for (const auto& mapping : mapping_.effector) {
            const auto joint = world_->Joint(mapping.joint);
            const auto actuator = world_->Actuator(mapping.actuator);
            Require(actuator.affine_position_control && actuator.enabled,
                    "MuJoCo effector requires a stable affine position servo");
            Require(actuator.transmission_supported &&
                            std::find(actuator.controlled_joints.begin(),
                                    actuator.controlled_joints.end(),
                                    mapping.joint) != actuator.controlled_joints.end(),
                    "MuJoCo effector actuator does not control its mapped joint");
            for (const auto& joint_name : actuator.controlled_joints) {
                Require(controlled_joints.insert(joint_name).second,
                        "MuJoCo effector actuators control overlapping joints");
                for (const auto& arm : mapping_.arm) {
                    Require(arm.joint != joint_name,
                            "MuJoCo effector actuator also controls an arm joint");
                }
            }
            CheckJointRange(joint, mapping.joint_closed, mapping.joint_open);
            CheckControlRange(actuator, mapping.control_closed, mapping.control_open);
            const auto normalized = Normalize(joint.position, mapping, effector_config_->tolerance);
            const double control_span = std::abs(mapping.control_open - mapping.control_closed);
            Require(std::abs(actuator.control - Control(normalized, mapping)) <=
                                    1e-6 * std::max(1.0, control_span) &&
                            std::abs(
                                    joint.velocity / (mapping.joint_open - mapping.joint_closed)) <=
                                    mapping_.effector_moving_threshold,
                    "MuJoCo initial effector control/velocity would start an unrequested motion");
            if (actuator.joint_transmission) {
                Require(std::isfinite(actuator.gear) && std::abs(actuator.gear) > 1e-12,
                        "MuJoCo effector transmission has zero gear");
                const auto target = [&](double control) {
                    return -actuator.gain * control / (actuator.position_bias * actuator.gear);
                };
                Require(std::abs(target(mapping.control_closed) - mapping.joint_closed) <= 1e-6 &&
                                std::abs(target(mapping.control_open) - mapping.joint_open) <= 1e-6,
                        "MuJoCo direct effector endpoints disagree with actuator transmission");
            }
        }
    }
}

void SharedSession::Acquire() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    std::lock_guard<std::mutex> guard(mutex_);
    if (owners_ > 0) {
        CheckReady();
        ++owners_;
        return;
    }
    Require(!thread_.joinable(), "MuJoCo previous owner has not stopped");
    fault_.clear();
    stopping_ = false;
    arm_motion_ = {};
    effector_motion_ = {};
    try {
        world_ = std::make_unique<mujoco_sim::World>(
                std::filesystem::canonical(mapping_.scene).string());
        world_->Reset(mapping_.keyframe);
        ValidateModel();
        Sample();
        owners_ = 1;
        thread_ = std::thread(&SharedSession::Run, this);
    } catch (...) {
        owners_ = 0;
        world_.reset();
        throw;
    }
}

void SharedSession::Release() noexcept {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    bool last = false;
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (owners_ > 0) last = --owners_ == 0;
        if (last) stopping_ = true;
    }
    if (last) Stop();
}

void SharedSession::Stop() noexcept {
    {
        std::lock_guard<std::mutex> guard(mutex_);
        stopping_ = true;
        changed_.notify_all();
    }
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> guard(mutex_);
    world_.reset();
    owners_ = 0;
}

void SharedSession::CheckReady() const {
    Require(owners_ > 0 && world_ && !stopping_, "MuJoCo world disconnected");
    Require(fault_.empty(), "MuJoCo physics fault: " + fault_);
}

void SharedSession::Sample() {
    const auto received = Clock::now();
    const auto sample_id = ++sample_sequence_;
    if (arm_config_) {
        ArmState state;
        state.received_at = received;
        state.sample_id = sample_id;
        state.source_time_s = world_->Time();
        state.enabled = true;  // Scene position controllers, not a hardware torque operation.
        state.position_mode = true;
        state.command_active = arm_motion_.active;
        state.firmware = "mujoco-headless-local";
        for (const auto& mapping : mapping_.arm) {
            const auto joint = world_->Joint(mapping.joint);
            Require(std::isfinite(joint.position) && std::isfinite(joint.velocity),
                    "nonfinite MuJoCo arm state");
            state.joints_rad.push_back(joint.position);
            state.moving =
                    state.moving || std::abs(joint.velocity) > mapping_.arm_moving_threshold_rad_s;
        }
        state.valid = true;
        arm_sample_ = std::move(state);
    }
    if (effector_config_) {
        EffectorState state;
        state.received_at = received;
        state.sample_id = sample_id;
        state.enabled = true;
        state.side = "simulation";
        state.command_active = effector_motion_.active;
        for (const auto& mapping : mapping_.effector) {
            const auto joint = world_->Joint(mapping.joint);
            state.positions.push_back(
                    Normalize(joint.position, mapping, effector_config_->tolerance));
            Require(std::isfinite(joint.velocity), "nonfinite MuJoCo effector velocity");
            state.moving = state.moving ||
                    std::abs(joint.velocity / (mapping.joint_open - mapping.joint_closed)) >
                            mapping_.effector_moving_threshold;
        }
        state.valid = true;
        effector_sample_ = std::move(state);
    }
}

ArmState SharedSession::ReadArm() {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Require(arm_config_.has_value(), "MuJoCo session has no arm");
    return arm_sample_;
}

EffectorState SharedSession::ReadEffector() {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Require(effector_config_.has_value(), "MuJoCo session has no effector");
    return effector_sample_;
}

mujoco_sim::WorldSnapshot SharedSession::Snapshot() {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    return world_->Snapshot();
}

void SharedSession::Reset() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Reject(owners_ == 1, "disconnect control roles before resetting the runtime");
    world_->Reset(mapping_.keyframe);
    arm_motion_ = {};
    effector_motion_ = {};
    ValidateModel();
    Sample();
    changed_.notify_all();
}

void SharedSession::AttachWeld(const std::string& equality, const std::string& body1,
        const std::string& body2, const std::string& left_pad, const std::string& right_pad) {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Reject(!arm_motion_.active && !effector_motion_.active && !arm_sample_.moving &&
                    !effector_sample_.moving,
            "cannot attach while commanded/measured motion remains");
    Reject(left_pad != right_pad && world_->TouchingBodies(left_pad, body2) &&
                    world_->TouchingBodies(right_pad, body2),
            "weld requires both pad bodies contacting the same object");
    world_->AttachWeld(equality, body1, body2);
    Sample();
}

void SharedSession::ReleaseWeld(const std::string& equality) {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Reject(!arm_motion_.active && !arm_sample_.moving, "cannot release while arm is moving");
    world_->ReleaseWeld(equality);
    Sample();
}

void SharedSession::SubmitArm(const JointPoint& point) {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Reject(arm_config_ && point.target_rad.size() == mapping_.arm.size(),
            "invalid MuJoCo target dimensions");
    Reject(InRange(point.speed_scale, 0, 1) && point.speed_scale > 0, "invalid MuJoCo speed scale");
    Reject(point.resend_interval_s == 0, "MuJoCo does not support target resend intervals");
    for (std::size_t index = 0; index < point.target_rad.size(); ++index) {
        const auto& limit = arm_config_->joints[index];
        Reject(InRange(point.target_rad[index], limit.minimum_rad, limit.maximum_rad),
                "MuJoCo target outside configured limits");
    }
    if (!arm_motion_.initialized) arm_motion_.reference = arm_sample_.joints_rad;
    arm_motion_.target = point.target_rad;
    arm_motion_.rates.clear();
    for (const auto& mapping : mapping_.arm) {
        arm_motion_.rates.push_back(mapping.max_velocity_rad_s * point.speed_scale);
    }
    if (mapping_.synchronized_joint_point) {
        double duration = 0;
        for (std::size_t index = 0; index < point.target_rad.size(); ++index) {
            duration = std::max(duration,
                    std::abs(point.target_rad[index] - arm_motion_.reference[index]) /
                            arm_motion_.rates[index]);
        }
        for (std::size_t index = 0; index < point.target_rad.size(); ++index) {
            arm_motion_.rates[index] = duration > 0
                    ? std::abs(point.target_rad[index] - arm_motion_.reference[index]) / duration
                    : 0;
        }
    }
    arm_motion_.initialized = true;
    arm_motion_.active = true;
}

void SharedSession::SubmitEffector(const EffectorPoint& point) {
    std::lock_guard<std::mutex> guard(mutex_);
    CheckReady();
    Reject(effector_config_ && point.positions.size() == mapping_.effector.size(),
            "invalid MuJoCo effector dimensions");
    Reject(InRange(point.duration_s, 0, 120) && point.duration_s > 0,
            "invalid MuJoCo effector duration");
    Reject(point.duration_s >= world_->Timestep(),
            "MuJoCo effector duration is shorter than one physics step");
    for (const double position : point.positions) {
        Reject(InRange(position, 0, 1), "MuJoCo effector targets must be normalized");
    }
    if (!effector_motion_.initialized) effector_motion_.reference = effector_sample_.positions;
    effector_motion_.target = point.positions;
    effector_motion_.rates.clear();
    for (std::size_t index = 0; index < point.positions.size(); ++index) {
        effector_motion_.rates.push_back(
                std::abs(point.positions[index] - effector_motion_.reference[index]) /
                point.duration_s);
    }
    effector_motion_.initialized = true;
    effector_motion_.active = true;
}

void SharedSession::Advance(Motion* motion, bool arm, double timestep) {
    if (!motion->active) return;
    std::vector<std::pair<std::string, double>> controls;
    bool active = false;
    for (std::size_t index = 0; index < motion->target.size(); ++index) {
        const double difference = motion->target[index] - motion->reference[index];
        const double step = motion->rates[index] * timestep;
        if (std::abs(difference) <= step) {
            motion->reference[index] = motion->target[index];
        } else {
            motion->reference[index] += std::copysign(step, difference);
            active = true;
        }
        const auto& actuator =
                arm ? mapping_.arm[index].actuator : mapping_.effector[index].actuator;
        double control = motion->reference[index];
        if (!arm) control = Control(motion->reference[index], mapping_.effector[index]);
        controls.emplace_back(actuator, control);
    }
    world_->SetControls(controls);
    motion->active = active;
}

void SharedSession::Run() noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    try {
        const double timestep = world_->Timestep();
        const auto tick = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(timestep));
        auto deadline = Clock::now() + tick;
        double advance_ms = 0;
        double step_ms = 0;
        double sample_ms = 0;
        ThreadUsage previous_usage;
        const auto milliseconds = [](Clock::duration duration) {
            return std::chrono::duration<double, std::milli>(duration).count();
        };
        while (!stopping_) {
            const auto wait_started = Clock::now();
            if (changed_.wait_until(lock, deadline, [&] { return stopping_; })) break;
            const auto awakened = Clock::now();
            // Never burst through unbounded catch-up steps after a blocked host scheduler.
            if (awakened - deadline > std::chrono::milliseconds(100)) {
                std::string usage_detail =
                        " previous_cpu_ms=unavailable"
                        " previous_minor_faults=unavailable"
                        " previous_major_faults=unavailable";
                if (previous_usage.valid) {
                    usage_detail = " previous_cpu_ms=" + std::to_string(previous_usage.cpu_ms) +
                            " previous_minor_faults=" +
                            std::to_string(previous_usage.minor_faults) +
                            " previous_major_faults=" + std::to_string(previous_usage.major_faults);
                }
                throw std::runtime_error("real-time physics scheduling overrun: late_ms=" +
                        std::to_string(milliseconds(awakened - deadline)) +
                        " wait_ms=" + std::to_string(milliseconds(awakened - wait_started)) +
                        " previous_advance_ms=" + std::to_string(advance_ms) +
                        " previous_step_ms=" + std::to_string(step_ms) +
                        " previous_sample_ms=" + std::to_string(sample_ms) + usage_detail);
            }
            // Per-thread counters distinguish actual physics work from scheduling and paging.
            // Collection is allocation-free; unsupported platforms retain wall-time diagnostics.
            const auto usage_started = ReadThreadUsage();
            Advance(&arm_motion_, true, timestep);
            Advance(&effector_motion_, false, timestep);
            const auto advanced = Clock::now();
            world_->Step();
            const auto stepped = Clock::now();
            Sample();
            const auto sampled = Clock::now();
            const auto usage_finished = ReadThreadUsage();
            previous_usage.valid = usage_started.valid && usage_finished.valid;
            if (previous_usage.valid) {
                previous_usage.cpu_ms = usage_finished.cpu_ms - usage_started.cpu_ms;
                previous_usage.minor_faults =
                        usage_finished.minor_faults - usage_started.minor_faults;
                previous_usage.major_faults =
                        usage_finished.major_faults - usage_started.major_faults;
            }
            advance_ms = milliseconds(advanced - awakened);
            step_ms = milliseconds(stepped - advanced);
            sample_ms = milliseconds(sampled - stepped);
            changed_.notify_all();
            deadline += tick;
        }
    } catch (const std::exception& error) {
        fault_ = error.what();
    } catch (...) {
        fault_ = "unknown physics failure";
    }
    changed_.notify_all();
}

bool SharedSession::HoldAll() {
    std::unique_lock<std::mutex> lock(mutex_);
    CheckReady();
    const auto now = Clock::now();
    if (arm_config_) {
        Require(std::chrono::duration<double>(now - arm_sample_.received_at).count() <=
                        arm_config_->maximum_age_s,
                "MuJoCo hold requires fresh arm feedback");
    }
    if (effector_config_) {
        Require(std::chrono::duration<double>(now - effector_sample_.received_at).count() <=
                        effector_config_->maximum_age_s,
                "MuJoCo hold requires fresh effector feedback");
    }

    // Both interpolators are stopped under the same world lock, and SetControls validates the
    // entire batch before writing. No role keeps chasing its old target while another settles.
    std::vector<std::pair<std::string, double>> controls;
    if (arm_config_) {
        arm_motion_.reference = arm_sample_.joints_rad;
        arm_motion_.target = arm_motion_.reference;
        arm_motion_.active = false;
        arm_motion_.initialized = true;
        for (std::size_t index = 0; index < mapping_.arm.size(); ++index) {
            controls.emplace_back(mapping_.arm[index].actuator, arm_motion_.reference[index]);
        }
    }
    if (effector_config_) {
        effector_motion_.reference = effector_sample_.positions;
        effector_motion_.target = effector_motion_.reference;
        effector_motion_.active = false;
        effector_motion_.initialized = true;
        for (std::size_t index = 0; index < mapping_.effector.size(); ++index) {
            controls.emplace_back(mapping_.effector[index].actuator,
                    Control(effector_motion_.reference[index], mapping_.effector[index]));
        }
    }
    world_->SetControls(controls);
    const auto submitted = Clock::now();
    const auto deadline = submitted +
            std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(mapping_.hold_timeout_s));
    auto previous = sample_sequence_;
    unsigned arm_settled = 0;
    unsigned effector_settled = 0;
    while (Clock::now() < deadline) {
        changed_.wait_until(lock, deadline,
                [&] { return sample_sequence_ > previous || stopping_ || !fault_.empty(); });
        if (stopping_ || !fault_.empty() || Clock::now() >= deadline) return false;
        if (sample_sequence_ <= previous) continue;
        previous = sample_sequence_;
        const auto sampled = Clock::now();
        if (arm_config_) {
            if (std::chrono::duration<double>(sampled - arm_sample_.received_at).count() >
                    arm_config_->maximum_age_s)
                return false;
            if (arm_sample_.received_at > submitted) {
                arm_settled =
                        arm_sample_.moving || arm_sample_.command_active ? 0 : arm_settled + 1;
            }
        }
        if (effector_config_) {
            if (std::chrono::duration<double>(sampled - effector_sample_.received_at).count() >
                    effector_config_->maximum_age_s)
                return false;
            if (effector_sample_.received_at > submitted) {
                effector_settled = effector_sample_.moving || effector_sample_.command_active
                        ? 0
                        : effector_settled + 1;
            }
        }
        if ((!arm_config_ || arm_settled >= 3) && (!effector_config_ || effector_settled >= 3))
            return true;
    }
    return false;
}

bool SharedSession::Hold(bool arm) {
    std::unique_lock<std::mutex> lock(mutex_);
    CheckReady();
    Require(arm ? arm_config_.has_value() : effector_config_.has_value(),
            "MuJoCo hold role unavailable");
    auto& motion = arm ? arm_motion_ : effector_motion_;
    const auto received = arm ? arm_sample_.received_at : effector_sample_.received_at;
    const double maximum_age = arm ? arm_config_->maximum_age_s : effector_config_->maximum_age_s;
    Require(std::chrono::duration<double>(Clock::now() - received).count() <= maximum_age,
            "MuJoCo hold requires fresh feedback");
    motion.reference = arm ? arm_sample_.joints_rad : effector_sample_.positions;
    motion.target = motion.reference;
    motion.active = false;
    motion.initialized = true;
    std::vector<std::pair<std::string, double>> controls;
    for (std::size_t index = 0; index < motion.reference.size(); ++index) {
        if (arm) {
            controls.emplace_back(mapping_.arm[index].actuator, motion.reference[index]);
        } else {
            controls.emplace_back(mapping_.effector[index].actuator,
                    Control(motion.reference[index], mapping_.effector[index]));
        }
    }
    world_->SetControls(controls);
    const auto submitted = Clock::now();
    const auto deadline = submitted +
            std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(mapping_.hold_timeout_s));
    auto previous = arm ? arm_sample_.sample_id : effector_sample_.sample_id;
    unsigned settled = 0;
    while (Clock::now() < deadline) {
        changed_.wait_until(lock, deadline, [&] {
            const auto sample = arm ? arm_sample_.sample_id : effector_sample_.sample_id;
            return sample > previous || stopping_ || !fault_.empty();
        });
        if (stopping_ || !fault_.empty()) return false;
        const auto sample = arm ? arm_sample_.sample_id : effector_sample_.sample_id;
        const auto sampled = arm ? arm_sample_.received_at : effector_sample_.received_at;
        const auto now = Clock::now();
        if (now >= deadline || std::chrono::duration<double>(now - sampled).count() > maximum_age)
            return false;
        if (sample <= previous || sampled <= submitted) continue;
        previous = sample;
        const bool moving = arm ? arm_sample_.moving : effector_sample_.moving;
        settled = moving ? 0 : settled + 1;
        if (settled >= 3) return true;
    }
    return false;
}
}  // namespace spacemit::manipulation::simulation
