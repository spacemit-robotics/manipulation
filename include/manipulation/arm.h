// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Arm driver contract. No end-effector, vendor types or implicit home/enable.
#ifndef MANIPULATION_ARM_H_
#define MANIPULATION_ARM_H_

#include <string>
#include <vector>

#include "manipulation/types.h"

namespace spacemit::manipulation {

// Limits the controller itself reports and enforces (may be narrower than the configured ones).
struct ControllerJointLimit {
    double minimum_rad = 0;
    double maximum_rad = 0;
    double max_velocity_rad_s = 0;
    double max_acceleration_rad_s2 = 0;
};

struct ArmState {
    std::vector<double> joints_rad;
    Clock::time_point received_at{};  // Oldest receipt in this complete joint/status sample.
    std::uint64_t sample_id = 0;
    double source_time_s = 0;
    bool valid = false;
    bool enabled = false;
    bool position_mode = false;
    bool moving = false;
    bool fault = false;
    std::string firmware;
    std::vector<ControllerJointLimit> controller_limits;  // Read by Diagnose(); empty when unknown.
    // The driver is still delivering the last command (e.g. streaming).
    bool command_active = false;
};

// Controller speed ratio, NOT a physical velocity bound in rad/s.
// Drivers must reject constraints they cannot guarantee, never silently reinterpret units.
// wait_for_arrival false: complete on two fresh samples within tolerance_rad, before the arm is
// still; it may still be settling, and the next command may start from wherever it is.
// resend_interval_s (0, or 0.02..0.2 s): re-send the same target at this interval while beyond
// tolerance_rad, for drivers that can lose a target; 0 keeps the driver's own policy.
struct JointPoint {
    std::vector<double> target_rad;
    double speed_scale = 0.05;
    // Arrival tolerance; 0: the device's tolerance_rad (at most 0.8 deg).
    double tolerance_rad = 0;
    bool wait_for_arrival = true;
    double resend_interval_s = 0;
};

// Continuous motion through waypoints from the current position, stopping only at the last.
// The joint-space profile is smooth (zero velocity at both ends); its velocity and acceleration
// bound the largest joint change (rad/s, rad/s^2). speed_scale is the controller ratio, as above.
// Arrival at the last waypoint: within tolerance_rad on every joint (0: the device's tolerance_rad;
// at most 0.8 deg, for drivers whose streaming mode leaves a small steady-state error) and still.
struct JointPath {
    std::vector<std::vector<double>> waypoints_rad;
    double max_velocity_rad_s = 0.2;
    double max_acceleration_rad_s2 = 0.2;
    double speed_scale = 0.05;
    double tolerance_rad = 0;
};

// A timed joint trajectory: knots (times_s[i], positions_rad[i]) from times_s[0] = 0, interpolated
// linearly and streamed at command_rate_hz. The first knot must be the current position; no
// commanded step may exceed max_step_rad. Arrival at the last knot as for JointPath, or, with
// wait_for_arrival false, completion once the last knot is delivered: the arm may still be
// settling, and the next command may start from wherever it is.
struct JointTrajectory {
    std::vector<double> times_s;
    std::vector<std::vector<double>> positions_rad;
    double command_rate_hz = 100;
    double max_step_rad = 0.0174533;  // 1 degree
    double speed_scale = 1.0;
    double tolerance_rad = 0;
    bool wait_for_arrival = true;
};

struct ArmCapabilities {
    bool joint_point = false;
    bool enable = false;
    bool verified_hold = false;
    double minimum_speed_scale = 0;  // Driver constraint; strict positivity still required.
    bool joint_path = false;
    bool joint_trajectory = false;
    bool disable = false;  // Explicit torque release, never used by Hold().
};

// Nominal duration of a JointPath profile starting at start_rad (s).
double JointPathDuration(const std::vector<double>& start_rad, const JointPath& path);

// Called by one owner at a time. Throw CommandRejected only when nothing was written.
class Arm {
public:
    virtual ~Arm() = default;
    virtual ArmCapabilities GetCapabilities() const = 0;
    virtual void Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    virtual ArmState Read() = 0;
    virtual ArmState Diagnose();  // Explicit active parameter/firmware queries, no motion.
    virtual void Enable();        // Explicit operation; default: unsupported.
    virtual void Disable();       // Explicit torque release; default: rejected without a write.
    virtual void Submit(const JointPoint& point) = 0;  // Acceptance, not completion.
    // Acceptance; default: rejected, nothing written.
    virtual void SubmitPath(const JointPath& path);
    virtual void SubmitTrajectory(const JointTrajectory& trajectory);  // Same; default: rejected.
    virtual bool Hold() = 0;  // True only after distinct fresh feedback confirms stop.
};

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_ARM_H_
