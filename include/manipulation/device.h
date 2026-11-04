// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// A single owned arm with an optional independent end effector. No vendor types.
#ifndef DEVICE_H
#define DEVICE_H

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/arm.h"

namespace spacemit::manipulation {
// kNone: the deployment has no position/status feedback. Moves are delivered open loop and
// reported as kUnverified; a stop can never be confirmed. Only the liveness sample is checked.
enum class EffectorFeedback { kPosition, kNone };
struct EffectorConfig {
    std::string device_id;
    std::vector<std::string> channels;
    EffectorFeedback feedback = EffectorFeedback::kPosition;
    double tolerance = 0.02;  // Normalized channel position, not force or radians.
    double timeout_s = 5;
    double maximum_age_s = 0.2;
};
struct EffectorState {
    std::vector<double> positions;  // Measured; empty when the effector has no position feedback.
    Clock::time_point received_at{};
    std::uint64_t sample_id = 0;
    bool valid = false;
    bool moving = false;
    bool fault = false;
    std::string side;
    std::optional<bool> enabled = std::nullopt;  // Unknown when the device does not report torque state.
    // Per-channel load in [-1,1] of the device's rated output (sign: drive direction); empty if unknown.
    std::vector<double> loads{};
    // Grasp evidence from a driver with grasp_detection: stopped short of a closing target on load.
    std::optional<bool> holding = std::nullopt;
};
struct EffectorPoint {
    std::vector<double> positions;  // Per-channel normalized: 0 closed, 1 open, on every end effector.
    double duration_s = 0.5;
};
// An effector target sent at_s after an arm trajectory starts, while the arm moves: an arm-and-hand
// gesture. Targets are sent in order, at most one per control cycle, and never verified.
struct TimedEffectorPoint {
    double at_s = 0;
    EffectorPoint point;
};
struct EffectorCapabilities {
    bool verified_hold = false;
    double maximum_duration_s = 120;
    bool enable = false;
    bool disable = false;
    // The driver reports holding: a closing move that stops on an object completes as a grasp.
    bool grasp_detection = false;
};
class EndEffector {
public:
    virtual ~EndEffector() = default;
    virtual EffectorCapabilities GetCapabilities() const { return {}; }
    virtual void Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    virtual EffectorState Read() = 0;
    virtual void Enable();  // Explicit operation; default: rejected without a write.
    virtual void Disable();  // Explicit torque release; never used to hold or cancel.
    virtual void Submit(const EffectorPoint& point) = 0;
    virtual bool Hold() { return false; }  // Unsupported is never a verified stop.
};
struct DeviceDefinition {
    DeviceConfig arm;
    EffectorConfig effector;  // Empty channels means arm-only.
};
struct Devices {
    std::unique_ptr<Arm> arm;
    std::unique_ptr<EndEffector> effector;
};
struct DeviceDriver {
    std::string id;
    unsigned interface_version = 1;
    std::function<void(const DeviceDefinition&, const DriverOptions&)> validate;
    std::function<Devices(const DeviceDefinition&, const DriverOptions&)> create;
    double minimum_arm_speed_scale = 0;
    double maximum_effector_duration_s = 120;
    // Optional offline normalization; directory is the owning configuration directory.
    std::function<DriverOptions(const DriverOptions&, const std::string&)> resolve_options{};
};
class DeviceRegistry {
public:
    void Register(DeviceDriver driver);
    const DeviceDriver& Get(const std::string& id) const;
    std::vector<std::string> List() const;
    void Validate(const std::string& id, const DeviceDefinition& definition,
        const DriverOptions& options) const;
    Devices Create(const std::string& id, const DeviceDefinition& definition,
        const DriverOptions& options) const;
private:
    std::map<std::string, DeviceDriver> drivers_;
};
DeviceRegistry MakeBuiltinDeviceRegistry();
void ValidateEffector(const EffectorConfig& config);

// Serialized calls. One host-wide lease per arm resource_id covers the whole assembly.
// Failure latches the whole assembly; reconnect never enables/homes/replays a task.
// A CommandRejected from a move means nothing was written and nothing is latched.
class DeviceSession {
public:
    // Lease in DefaultLockDirectory(), shared by every tool on this host.
    DeviceSession(DeviceDefinition definition, Devices devices);
    // Explicit lease directory for isolated tests or a dedicated deployment layout.
    DeviceSession(DeviceDefinition definition, Devices devices, std::string lock_directory);
    ~DeviceSession();
    void Connect();
    void Disconnect() noexcept;
    ArmState ReadArm();
    EffectorState ReadEffector();
    void Diagnose();
    ArmCapabilities GetArmCapabilities() const;  // No connection or device write required.
    EffectorCapabilities GetEffectorCapabilities() const;  // Rejects arm-only assemblies.
    void EnableArm();  // Explicit opt-in only; no implicit enable on connection or motion.
    void DisableArm();  // Explicit torque release; unsupported drivers reject without writing.
    void EnableEffector();
    void DisableEffector();
    // Hold preserves torque. Fault latches permit an attempt but are never cleared by success.
    CommandResult HoldArm();
    CommandResult HoldEffector();
    // Relative step of one zero-based joint, using the same limits and validation as MoveArm.
    CommandResult JogArm(std::size_t joint_index, double delta_rad, double speed_scale = 0.05,
        const std::function<bool()>& cancelled = {});
    CommandResult MoveArm(const JointPoint& point, const std::function<bool()>& cancelled = {});
    // Target following, for tracking a moving target at a high rate: sends the target and returns without
    // waiting for arrival. The first target needs a stationary arm; later ones may replace a target the
    // arm is still moving to. Finish with HoldArm (stop where it is) or MoveArm (arrive at a final target).
    CommandResult FollowArm(const JointPoint& point);
    // Continuous motion through the waypoints (drivers with the joint_path capability).
    CommandResult MoveArmPath(const JointPath& path, const std::function<bool()>& cancelled = {});
    // Timed trajectory, streamed as given (drivers with the joint_trajectory capability). With effector
    // targets (at most 64, at_s ordered within 60 s), the command completes once the trajectory has and
    // every target has been sent and had its duration_s; the result is then unverified (not measured).
    CommandResult MoveArmTrajectory(const JointTrajectory& trajectory, const std::function<bool()>& cancelled = {},
        const std::vector<TimedEffectorPoint>& effector = {});
    CommandResult MoveEffector(const EffectorPoint& point, const std::function<bool()>& cancelled = {});
    // As FollowArm for the end effector. The state reports position and grasp evidence meanwhile; without
    // position feedback the result is unverified.
    CommandResult FollowEffector(const EffectorPoint& point);
    bool HasEffector() const;
    bool Latched() const;
    const DeviceDefinition& Definition() const;
private:
    void Ready() const;
    CommandResult RunArm(const std::function<CommandResult()>& submit, const std::function<bool()>& cancelled,
        const std::vector<TimedEffectorPoint>& effector = {});
    void CheckEffectorPoint(const EffectorPoint& point) const;
    void SetEffectorEnabled(bool enabled);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace spacemit::manipulation
#endif  // DEVICE_H
