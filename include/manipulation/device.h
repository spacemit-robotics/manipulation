// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// A single owned arm with an optional independent end effector. No vendor types.
#ifndef DEVICE_H
#define DEVICE_H

#include <functional>
#include <map>
#include <memory>
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
};
struct EffectorPoint {
    std::vector<double> positions;  // Per-channel normalized [0,1].
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
};
class EndEffector {
public:
    virtual ~EndEffector() = default;
    virtual EffectorCapabilities GetCapabilities() const { return {}; }
    virtual void Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    virtual EffectorState Read() = 0;
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
    void EnableArm();  // Explicit opt-in only. No disable/clear/home API.
    CommandResult MoveArm(const JointPoint& point, const std::function<bool()>& cancelled = {});
    // Continuous motion through the waypoints (drivers with the joint_path capability).
    CommandResult MoveArmPath(const JointPath& path, const std::function<bool()>& cancelled = {});
    // Timed trajectory, streamed as given (drivers with the joint_trajectory capability). With effector
    // targets (at most 64, at_s ordered within 60 s), the command completes once the trajectory has and
    // every target has been sent and had its duration_s; the result is then unverified (not measured).
    CommandResult MoveArmTrajectory(const JointTrajectory& trajectory, const std::function<bool()>& cancelled = {},
        const std::vector<TimedEffectorPoint>& effector = {});
    CommandResult MoveEffector(const EffectorPoint& point, const std::function<bool()>& cancelled = {});
    bool HasEffector() const;
    bool Latched() const;
    const DeviceDefinition& Definition() const;
private:
    void Ready() const;
    CommandResult RunArm(const std::function<CommandResult()>& submit, const std::function<bool()>& cancelled,
        const std::vector<TimedEffectorPoint>& effector = {});
    void CheckEffectorPoint(const EffectorPoint& point) const;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace spacemit::manipulation
#endif  // DEVICE_H
