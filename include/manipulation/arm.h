// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file arm.h
 * @brief Arm driver contract and motion requests.
 *
 * No end effector, vendor types or implicit home or enable.
 */
#ifndef MANIPULATION_ARM_H_
#define MANIPULATION_ARM_H_

#include <string>
#include <vector>

#include "manipulation/types.h"

namespace spacemit::manipulation {

/// Limits the controller itself reports and enforces; may be narrower than the configured ones.
struct ControllerJointLimit {
    /// Lower position limit, rad.
    double minimum_rad = 0;
    /// Upper position limit, rad.
    double maximum_rad = 0;
    /// Velocity limit, rad/s.
    double max_velocity_rad_s = 0;
    /// Acceleration limit, rad/s^2.
    double max_acceleration_rad_s2 = 0;
};

/// One complete arm feedback sample.
struct ArmState {
    /// Measured joint positions in configuration order, rad.
    std::vector<double> joints_rad;
    /// Oldest receipt in this complete joint and status sample.
    Clock::time_point received_at{};
    /// Increases with every new sample, so repeated reads can be told apart.
    std::uint64_t sample_id = 0;
    /// Device timestamp of the sample, s; 0 when the device has none.
    double source_time_s = 0;
    /// True when every field comes from fresh, complete feedback.
    bool valid = false;
    /// Joint motors are enabled.
    bool enabled = false;
    /// The controller accepts position targets.
    bool position_mode = false;
    /// The arm is measured to be moving.
    bool moving = false;
    /// The controller reports a fault.
    bool fault = false;
    /// Firmware description reported by Diagnose().
    std::string firmware;
    /// Controller limits read by Diagnose(); empty when unknown.
    std::vector<ControllerJointLimit> controller_limits;
    /// The driver is still delivering the last command, for example while streaming.
    bool command_active = false;
};

/**
 * @brief Joint point move.
 *
 * speed_scale is a controller speed ratio in (0, 1], not a velocity in rad/s. Drivers reject
 * constraints they cannot guarantee and never reinterpret units.
 */
struct JointPoint {
    /// Target per joint in configuration order, rad.
    std::vector<double> target_rad;
    /// Controller speed ratio in (0, 1].
    double speed_scale = 0.05;
    /// Arrival tolerance, at most 0.8 deg; 0 uses the configured tolerance_rad.
    double tolerance_rad = 0;
    /// True: complete when three fresh samples are within tolerance and still. False: complete on
    /// two fresh samples within tolerance; the arm may still be settling and the next command may
    /// start from wherever it is.
    bool wait_for_arrival = true;
    /// 0, or 0.02 to 0.2 s: re-send the same target at this interval while beyond tolerance, for
    /// drivers that can lose a target; 0 keeps the driver's own policy.
    double resend_interval_s = 0;
};

/**
 * @brief Continuous motion through waypoints from the current position, stopping only at the last.
 *
 * The joint-space profile is smooth, with zero velocity at both ends; its velocity and
 * acceleration bound the largest joint change. Arrival at the last waypoint means every joint is
 * within tolerance and still.
 */
struct JointPath {
    /// Waypoints in configuration order, rad; at most 128.
    std::vector<std::vector<double>> waypoints_rad;
    /// Velocity bound of the largest joint change, rad/s, at most 0.35.
    double max_velocity_rad_s = 0.2;
    /// Acceleration bound of the largest joint change, rad/s^2, at most 1.0.
    double max_acceleration_rad_s2 = 0.2;
    /// Controller speed ratio in (0, 1].
    double speed_scale = 0.05;
    /// Arrival tolerance at the last waypoint, at most 0.8 deg; 0 uses the configured tolerance.
    double tolerance_rad = 0;
};

/**
 * @brief Timed joint trajectory, streamed at a fixed rate.
 *
 * Knots (times_s[i], positions_rad[i]) start at times_s[0] = 0 and are interpolated linearly. The
 * first knot must be the current position, and no commanded step may exceed max_step_rad.
 */
struct JointTrajectory {
    /// Knot times from 0, s; increasing, at most 60 s and 4000 knots.
    std::vector<double> times_s;
    /// Joint positions at each knot, rad.
    std::vector<std::vector<double>> positions_rad;
    /// Streaming rate, 20 to 500 Hz.
    double command_rate_hz = 100;
    /// Largest commanded step between two cycles, rad (default 1 degree, at most 3 degrees).
    double max_step_rad = 0.0174533;
    /// Controller speed ratio in (0, 1].
    double speed_scale = 1.0;
    /// Arrival tolerance at the last knot, at most 0.8 deg; 0 uses the configured tolerance.
    double tolerance_rad = 0;
    /// True: complete on arrival at the last knot. False: complete once the last knot is sent; the
    /// arm may still be settling and the next command may start from wherever it is.
    bool wait_for_arrival = true;
};

/// Operations a driver supports. Configuration cannot add a capability the driver lacks.
struct ArmCapabilities {
    /// Accepts JointPoint targets.
    bool joint_point = false;
    /// Supports Enable().
    bool enable = false;
    /// Hold() confirms the stop from feedback.
    bool verified_hold = false;
    /// Smallest accepted speed ratio; requests must still be strictly positive.
    double minimum_speed_scale = 0;
    /// Accepts JointPath requests.
    bool joint_path = false;
    /// Accepts JointTrajectory requests.
    bool joint_trajectory = false;
    /// Supports Disable(), an explicit torque release never used by Hold().
    bool disable = false;
};

/**
 * @brief Nominal duration of a JointPath profile.
 *
 * @param start_rad Start position per joint, rad.
 * @param path Path whose waypoints, velocity and acceleration bounds set the profile.
 * @return Duration, s; at least 0.3 s.
 */
double JointPathDuration(const std::vector<double>& start_rad, const JointPath& path);

/**
 * @brief Arm driver interface.
 *
 * Called by one owner at a time. Methods throw CommandRejected only when nothing was written to
 * the device; any other exception from a write may mean partial execution.
 */
class Arm {
public:
    virtual ~Arm() = default;
    /// Operations this driver supports; needs no connection.
    virtual ArmCapabilities GetCapabilities() const = 0;
    /// Open the connection and read the state; never enables or moves the arm.
    virtual void Connect() = 0;
    /// Close the connection; never throws.
    virtual void Disconnect() noexcept = 0;
    /// Latest complete feedback sample.
    virtual ArmState Read() = 0;
    /// Query firmware and controller parameters explicitly; no motion.
    virtual ArmState Diagnose();
    /// Enable the joint motors; default: unsupported.
    virtual void Enable();
    /// Release torque explicitly; default: rejected without a write.
    virtual void Disable();
    /// Accept a point target; returns on acceptance, not completion.
    virtual void Submit(const JointPoint& point) = 0;
    /// Accept a path; default: rejected, nothing written.
    virtual void SubmitPath(const JointPath& path);
    /// Accept a trajectory; default: rejected, nothing written.
    virtual void SubmitTrajectory(const JointTrajectory& trajectory);
    /// Stop where the arm is; true only after distinct fresh feedback confirms the stop.
    virtual bool Hold() = 0;
};

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_ARM_H_
