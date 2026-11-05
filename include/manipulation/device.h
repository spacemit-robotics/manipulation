// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file device.h
 * @brief A single owned arm with an optional independent end effector, and the device session.
 *
 * No vendor types. End-effector positions are normalized per channel: 0 closed, 1 open, on every
 * end effector.
 */
#ifndef MANIPULATION_DEVICE_H_
#define MANIPULATION_DEVICE_H_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/arm.h"

namespace spacemit::manipulation {

/**
 * @brief Feedback the end-effector deployment provides.
 *
 * kNone: no position or status feedback. Moves are delivered open loop and reported as
 * kUnverified; a stop can never be confirmed. Only the liveness sample is checked.
 */
enum class EffectorFeedback { kPosition, kNone };

/// End-effector section of a device configuration.
struct EffectorConfig {
    /// Name of the end effector in logs and results.
    std::string device_id;
    /// Channels in position order, 1 to 32; empty means an arm-only device.
    std::vector<std::string> channels;
    /// Feedback the deployment provides.
    EffectorFeedback feedback = EffectorFeedback::kPosition;
    /// Arrival tolerance in normalized channel position, (0, 0.1]; not force or radians.
    double tolerance = 0.02;
    /// Deadline of one command, (0, 120] s.
    double timeout_s = 5;
    /// Oldest feedback sample accepted as current, (0, 1] s.
    double maximum_age_s = 0.2;
};

/// One end-effector feedback sample.
struct EffectorState {
    /// Measured positions per channel; empty without position feedback.
    std::vector<double> positions;
    /// Receipt time of the sample.
    Clock::time_point received_at{};
    /// Increases with every new sample.
    std::uint64_t sample_id = 0;
    /// True when the sample is fresh and complete.
    bool valid = false;
    /// The end effector is measured to be moving.
    bool moving = false;
    /// The end effector reports a fault.
    bool fault = false;
    /// Mounting side reported by the device, such as "right"; "unknown" when not reported.
    std::string side;
    /// Torque state; empty when the device does not report it.
    std::optional<bool> enabled = std::nullopt;
    /// Per-channel load in [-1, 1] of the rated output (sign: drive direction); empty if unknown.
    std::vector<double> loads{};
    /// Grasp evidence from a driver with grasp_detection: stopped short of a closing target under
    /// load. Empty for other drivers.
    std::optional<bool> holding = std::nullopt;
};

/// End-effector target.
struct EffectorPoint {
    /// Target per channel: 0 closed, 1 open.
    std::vector<double> positions;
    /// Execution time, s; bounded by the driver and the configured timeout_s.
    double duration_s = 0.5;
};

/**
 * @brief An end-effector target sent at_s after an arm trajectory starts, while the arm moves.
 *
 * Together they form an arm-and-hand gesture. Targets are sent in order, at most one per control
 * cycle, and never verified.
 */
struct TimedEffectorPoint {
    /// Send time after the trajectory starts, s.
    double at_s = 0;
    /// Target to send.
    EffectorPoint point;
};

/// Operations an end-effector driver supports.
struct EffectorCapabilities {
    /// Hold() confirms the stop from feedback.
    bool verified_hold = false;
    /// Longest accepted duration_s, s.
    double maximum_duration_s = 120;
    /// Supports Enable().
    bool enable = false;
    /// Supports Disable().
    bool disable = false;
    /// The driver reports holding: a closing move that stops on an object completes as a grasp.
    bool grasp_detection = false;
};

/**
 * @brief End-effector driver interface.
 *
 * Called by one owner at a time, with the same exception rules as Arm.
 */
class EndEffector {
public:
    virtual ~EndEffector() = default;

    /// Operations this driver supports; needs no connection.
    virtual EffectorCapabilities GetCapabilities() const { return {}; }

    /// Open the connection and read the state; never moves the end effector.
    virtual void Connect() = 0;
    /// Close the connection; never throws.
    virtual void Disconnect() noexcept = 0;
    /// Latest feedback sample.
    virtual EffectorState Read() = 0;
    /// Enable torque explicitly; default: rejected without a write.
    virtual void Enable();
    /// Release torque explicitly; never used to hold or cancel. Default: rejected.
    virtual void Disable();
    /// Accept a target; returns on acceptance, not completion.
    virtual void Submit(const EffectorPoint& point) = 0;

    /// Stop where it is; false when unsupported, which is never a verified stop.
    virtual bool Hold() { return false; }
};

/// Arm and optional end-effector configuration of one device.
struct DeviceDefinition {
    /// Arm configuration.
    DeviceConfig arm;
    /// End-effector configuration; empty channels means arm-only.
    EffectorConfig effector;
};

/// Driver instances created for one device.
struct Devices {
    /// Arm driver; never null.
    std::unique_ptr<Arm> arm;
    /// End-effector driver; null for an arm-only device.
    std::unique_ptr<EndEffector> effector;
};

/// A registered driver, selected by `device.driver` in a device configuration.
struct DeviceDriver {
    /// Registration name, such as "mock_device".
    std::string id;
    /// Driver interface version; must be 1.
    unsigned interface_version = 1;
    /// Check a definition and options offline; throws on invalid input.
    std::function<void(const DeviceDefinition&, const DriverOptions&)> validate;
    /// Create the driver instances; must not connect.
    std::function<Devices(const DeviceDefinition&, const DriverOptions&)> create;
    /// Smallest accepted arm speed ratio.
    double minimum_arm_speed_scale = 0;
    /// Longest accepted end-effector duration_s, s.
    double maximum_effector_duration_s = 120;
    /// Optional offline normalization of options, such as resolving paths against the
    /// directory of the configuration file.
    std::function<DriverOptions(const DriverOptions&, const std::string&)> resolve_options{};
};

/// Registered drivers by name.
class DeviceRegistry {
public:
    /**
     * @brief Register a driver.
     * @throws std::runtime_error If the driver is incomplete or its name is already registered.
     */
    void Register(DeviceDriver driver);
    /**
     * @brief Registered driver by name.
     * @throws std::runtime_error If no driver has this name.
     */
    const DeviceDriver& Get(const std::string& id) const;
    /// Names of the registered drivers.
    std::vector<std::string> List() const;
    /**
     * @brief Check a definition and options with the named driver, offline.
     * @throws std::runtime_error If the driver is unknown or rejects the configuration.
     */
    void Validate(const std::string& id, const DeviceDefinition& definition,
                  const DriverOptions& options) const;
    /**
     * @brief Validate, then create the driver instances; nothing is connected.
     * @throws std::runtime_error If validation fails or the driver returns no arm.
     */
    Devices Create(const std::string& id, const DeviceDefinition& definition,
                   const DriverOptions& options) const;

private:
    std::map<std::string, DeviceDriver> drivers_;
};

/// Registry with every driver built into this installation.
DeviceRegistry MakeBuiltinDeviceRegistry();

/**
 * @brief Check an end-effector configuration offline; an empty channel list is accepted.
 * @throws std::runtime_error If a field is outside its documented range.
 */
void ValidateEffector(const EffectorConfig& config);

/**
 * @brief Owns one device and executes commands with completion checks.
 *
 * Calls are serialized by the owner. One host-wide lease per arm resource_id covers the whole
 * assembly. A failure latches the whole assembly until Disconnect() and Connect(); reconnecting
 * never enables, homes or replays a task. A CommandRejected from a move means nothing was written
 * and nothing is latched.
 *
 * Every move accepts an optional `cancelled` callback, polled every 10 ms; returning true cancels
 * the command and holds the device.
 */
class DeviceSession {
public:
    /// Session with its lease in DefaultLockDirectory(), shared by every tool on this host.
    DeviceSession(DeviceDefinition definition, Devices devices);
    /// Session with an explicit lease directory, for isolated tests or a dedicated deployment.
    DeviceSession(DeviceDefinition definition, Devices devices, std::string lock_directory);
    ~DeviceSession();
    /// Acquire the lease, connect and read the state once; never enables or moves the device.
    void Connect();
    /// Hold a running command if any, then disconnect and release the lease; never throws.
    void Disconnect() noexcept;
    /// Fresh, complete arm state; throws when the sample is stale or incomplete.
    ArmState ReadArm();
    /// Fresh end-effector state; throws when the sample is stale or incomplete.
    EffectorState ReadEffector();
    /// Query firmware and controller parameters; no motion.
    void Diagnose();
    /// Arm operations of the driver; needs no connection or device write.
    ArmCapabilities GetArmCapabilities() const;
    /// End-effector operations of the driver; rejects arm-only assemblies.
    EffectorCapabilities GetEffectorCapabilities() const;
    /// Enable the arm explicitly; never done implicitly on connection or motion.
    void EnableArm();
    /// Release arm torque explicitly; unsupported drivers reject without writing.
    void DisableArm();
    /// Enable the end effector explicitly.
    void EnableEffector();
    /// Release end-effector torque explicitly.
    void DisableEffector();
    /**
     * @brief Hold the arm where it is, keeping torque.
     *
     * A latched session permits an attempt, but success never clears the latch.
     * @return kSucceeded when the stop is confirmed, otherwise kStopUnknown.
     */
    CommandResult HoldArm();
    /// As HoldArm() for the end effector.
    CommandResult HoldEffector();
    /**
     * @brief Relative step of one joint from its measured position.
     *
     * @param joint_index Zero-based joint index in configuration order.
     * @param delta_rad Step, rad; the target must stay within the joint limits.
     * @param speed_scale Controller speed ratio in (0, 1].
     * @param cancelled Optional cancellation callback.
     * @return Result with the same completion rules as MoveArm().
     */
    CommandResult JogArm(std::size_t joint_index, double delta_rad, double speed_scale = 0.05,
                         const std::function<bool()>& cancelled = {});
    /**
     * @brief Move to a joint point and block until completion, failure or cancellation.
     *
     * @return kSucceeded when the completion condition of the point is met.
     */
    CommandResult MoveArm(const JointPoint& point, const std::function<bool()>& cancelled = {});
    /**
     * @brief Target following, for tracking a moving target at a high rate.
     *
     * Sends the target and returns without waiting for arrival. The first target needs a
     * stationary arm; later ones may replace a target the arm is still moving to. Finish with
     * HoldArm() to stop where it is, or MoveArm() to arrive at a final target.
     * @return kSucceeded once the target is sent; detail starts with "sent".
     */
    CommandResult FollowArm(const JointPoint& point);
    /// Continuous motion through the waypoints (drivers with the joint_path capability).
    CommandResult MoveArmPath(const JointPath& path, const std::function<bool()>& cancelled = {});
    /**
     * @brief Timed trajectory, streamed as given (drivers with the joint_trajectory capability).
     *
     * @param trajectory Trajectory starting at the measured position.
     * @param cancelled Optional cancellation callback.
     * @param effector End-effector targets sent during the motion: at most 64, at_s ordered
     *        within 60 s. With targets, the command completes once the trajectory has and every
     *        target has been sent and had its duration_s; the result is then kUnverified.
     */
    CommandResult MoveArmTrajectory(const JointTrajectory& trajectory,
                                    const std::function<bool()>& cancelled = {},
                                    const std::vector<TimedEffectorPoint>& effector = {});
    /**
     * @brief Move the end effector and block until completion, failure or cancellation.
     *
     * @return kSucceeded on measured arrival or a detected grasp; kUnverified without position
     *         feedback.
     */
    CommandResult MoveEffector(const EffectorPoint& point,
                               const std::function<bool()>& cancelled = {});
    /**
     * @brief As FollowArm() for the end effector.
     *
     * ReadEffector() reports position and grasp evidence meanwhile; without position feedback the
     * result is kUnverified.
     */
    CommandResult FollowEffector(const EffectorPoint& point);
    /// True when the device has an end effector.
    bool HasEffector() const;
    /// True when a failure latched the session; only reconnecting clears it.
    bool Latched() const;
    /// Configuration the session was created with.
    const DeviceDefinition& Definition() const;

private:
    void Ready() const;
    CommandResult RunArm(const std::function<CommandResult()>& submit,
                         const std::function<bool()>& cancelled,
                         const std::vector<TimedEffectorPoint>& effector = {});
    void CheckEffectorPoint(const EffectorPoint& point) const;
    void SetEffectorEnabled(bool enabled);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_DEVICE_H_
