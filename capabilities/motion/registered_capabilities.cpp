// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Configurable motion capabilities over DeviceSession, independent of driver models.
#include "manipulation/capability.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "manipulation/device_plan.h"

namespace spacemit::manipulation {
namespace {
constexpr double kMaximumToleranceRad = 0.8 * 3.14159265358979323846 / 180;
constexpr double kMaximumTrajectoryStepRad = 3 * 3.14159265358979323846 / 180;

struct JointJog {
    std::size_t joint_index = 0;
    double delta_rad = 0;
    double speed_scale = 0.05;
};

void Require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

void Keys(const YAML::Node& node, const std::set<std::string>& allowed) {
    Require(node.IsMap(), "capability parameters must be a mapping");
    std::set<std::string> seen;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        if (!allowed.count(key) || !seen.insert(key).second)
            throw std::runtime_error("unknown/duplicate capability parameter: " + key);
    }
}

template <typename Value>
const Value& Parameters(const DeviceStep& step) {
    const auto* value = std::any_cast<Value>(&step.parameters);
    if (!value) throw std::runtime_error("invalid parameters for capability: " + step.capability);
    return *value;
}

void Speed(double speed_scale, const CapabilityContext& context) {
    Require(std::isfinite(speed_scale) && speed_scale > 0 && speed_scale <= 1 &&
                    speed_scale >= context.arm.minimum_speed_scale,
            "invalid or unsupported speed scale");
}

void Target(const std::vector<double>& target, const CapabilityContext& context) {
    const auto& joints = context.definition.arm.joints;
    Require(target.size() == joints.size(), "target joint count mismatch");
    for (std::size_t index = 0; index < joints.size(); ++index) {
        Require(std::isfinite(target[index]) && target[index] >= joints[index].minimum_rad &&
                        target[index] <= joints[index].maximum_rad,
                "joint limit exceeded");
    }
}

void Tolerance(double tolerance_rad) {
    Require(std::isfinite(tolerance_rad) && tolerance_rad >= 0 &&
                    tolerance_rad <= kMaximumToleranceRad,
            "tolerance must be in [0, 0.8 deg]");
}

std::string PointSupport(const CapabilityContext& context) {
    return context.arm.joint_point ? "" : "arm driver has no joint point control";
}

std::string EffectorSupport(const CapabilityContext& context) {
    return context.effector ? "" : "device has no end effector";
}

DeviceStep ParsePoint(const YAML::Node& node) {
    Keys(node,
            {"capability", "target_rad", "speed_scale", "tolerance_rad", "wait_for_arrival",
                    "resend_interval_s"});
    DeviceStep step;
    step.capability = "joint_point";
    step.arm.target_rad = node["target_rad"].as<std::vector<double>>();
    step.arm.speed_scale = node["speed_scale"].as<double>();
    if (node["tolerance_rad"]) step.arm.tolerance_rad = node["tolerance_rad"].as<double>();
    if (node["wait_for_arrival"]) step.arm.wait_for_arrival = node["wait_for_arrival"].as<bool>();
    if (node["resend_interval_s"])
        step.arm.resend_interval_s = node["resend_interval_s"].as<double>();
    return step;
}

void ValidatePoint(const DeviceStep& step, const CapabilityContext& context) {
    Target(step.arm.target_rad, context);
    Speed(step.arm.speed_scale, context);
    Tolerance(step.arm.tolerance_rad);
    Require(std::isfinite(step.arm.resend_interval_s) &&
                    (step.arm.resend_interval_s == 0 ||
                            (step.arm.resend_interval_s >= 0.02 &&
                                    step.arm.resend_interval_s <= 0.2)),
            "resend_interval_s must be 0 or in [0.02, 0.2] s");
}

DeviceStep ParseEffector(const YAML::Node& node) {
    Keys(node, {"capability", "positions", "duration_s"});
    DeviceStep step;
    step.capability = "effector_position";
    step.effector.positions = node["positions"].as<std::vector<double>>();
    step.effector.duration_s = node["duration_s"].as<double>();
    return step;
}

void ValidateEffectorPoint(const DeviceStep& step, const CapabilityContext& context) {
    const auto& point = step.effector;
    Require(point.positions.size() == context.definition.effector.channels.size(),
            "effector target channel count mismatch");
    Require(std::isfinite(point.duration_s) && point.duration_s > 0 &&
                    point.duration_s <= context.definition.effector.timeout_s &&
                    point.duration_s <= context.effector->maximum_duration_s,
            "invalid effector target or unsupported duration");
    for (double value : point.positions)
        Require(std::isfinite(value) && value >= 0 && value <= 1, "effector limit exceeded");
}

DeviceStep ParseJog(const YAML::Node& node) {
    Keys(node, {"capability", "joint_index", "delta_rad", "speed_scale"});
    const auto index = node["joint_index"].as<int>();
    Require(index >= 0, "joint_index must be nonnegative");
    JointJog jog;
    jog.joint_index = static_cast<std::size_t>(index);
    jog.delta_rad = node["delta_rad"].as<double>();
    if (node["speed_scale"]) jog.speed_scale = node["speed_scale"].as<double>();
    return {"joint_jog", {}, {}, jog};
}

void ValidateJog(const DeviceStep& step, const CapabilityContext& context) {
    const auto& jog = Parameters<JointJog>(step);
    Require(jog.joint_index < context.definition.arm.joints.size() && std::isfinite(jog.delta_rad),
            "invalid joint jog");
    Speed(jog.speed_scale, context);
    const auto& joint = context.definition.arm.joints[jog.joint_index];
    Require(std::abs(jog.delta_rad) <= joint.maximum_rad - joint.minimum_rad,
            "joint jog exceeds configured range");
    // The measured starting position and final joint bound are checked by JogArm at execution.
}

DeviceStep ParsePath(const YAML::Node& node) {
    Keys(node,
            {"capability", "waypoints_rad", "max_velocity_rad_s", "max_acceleration_rad_s2",
                    "speed_scale", "tolerance_rad"});
    JointPath path;
    path.waypoints_rad = node["waypoints_rad"].as<std::vector<std::vector<double>>>();
    if (node["max_velocity_rad_s"])
        path.max_velocity_rad_s = node["max_velocity_rad_s"].as<double>();
    if (node["max_acceleration_rad_s2"])
        path.max_acceleration_rad_s2 = node["max_acceleration_rad_s2"].as<double>();
    if (node["speed_scale"]) path.speed_scale = node["speed_scale"].as<double>();
    if (node["tolerance_rad"]) path.tolerance_rad = node["tolerance_rad"].as<double>();
    return {"joint_path", {}, {}, std::move(path)};
}

void ValidatePath(const DeviceStep& step, const CapabilityContext& context) {
    const auto& path = Parameters<JointPath>(step);
    Require(!path.waypoints_rad.empty() && path.waypoints_rad.size() <= 128,
            "joint path needs 1..128 waypoints");
    Require(std::isfinite(path.max_velocity_rad_s) && path.max_velocity_rad_s > 0 &&
                    path.max_velocity_rad_s <= 0.35,
            "path velocity must be in (0, 0.35] rad/s");
    Require(std::isfinite(path.max_acceleration_rad_s2) && path.max_acceleration_rad_s2 > 0 &&
                    path.max_acceleration_rad_s2 <= 1,
            "path acceleration must be in (0, 1] rad/s^2");
    Speed(path.speed_scale, context);
    Tolerance(path.tolerance_rad);
    for (const auto& waypoint : path.waypoints_rad) Target(waypoint, context);
}

DeviceStep ParseTrajectory(const YAML::Node& node) {
    Keys(node,
            {"capability", "times_s", "positions_rad", "command_rate_hz", "max_step_rad",
                    "speed_scale", "tolerance_rad", "wait_for_arrival"});
    JointTrajectory trajectory;
    trajectory.times_s = node["times_s"].as<std::vector<double>>();
    trajectory.positions_rad = node["positions_rad"].as<std::vector<std::vector<double>>>();
    if (node["command_rate_hz"]) trajectory.command_rate_hz = node["command_rate_hz"].as<double>();
    if (node["max_step_rad"]) trajectory.max_step_rad = node["max_step_rad"].as<double>();
    if (node["speed_scale"]) trajectory.speed_scale = node["speed_scale"].as<double>();
    if (node["tolerance_rad"]) trajectory.tolerance_rad = node["tolerance_rad"].as<double>();
    if (node["wait_for_arrival"]) trajectory.wait_for_arrival = node["wait_for_arrival"].as<bool>();
    return {"joint_trajectory", {}, {}, std::move(trajectory)};
}

void ValidateTrajectory(const DeviceStep& step, const CapabilityContext& context) {
    const auto& trajectory = Parameters<JointTrajectory>(step);
    const auto& times = trajectory.times_s;
    const auto& positions = trajectory.positions_rad;
    Require(positions.size() >= 2 && positions.size() <= 4000 && times.size() == positions.size(),
            "trajectory needs 2..4000 knots with one time each");
    Require(std::isfinite(trajectory.command_rate_hz) && trajectory.command_rate_hz >= 20 &&
                    trajectory.command_rate_hz <= 500,
            "command_rate_hz must be in [20, 500]");
    Require(std::isfinite(trajectory.max_step_rad) && trajectory.max_step_rad > 0 &&
                    trajectory.max_step_rad <= kMaximumTrajectoryStepRad * (1 + 1e-9),
            "max_step_rad must be in (0, 3 deg]");
    Require(times.front() == 0 && std::isfinite(times.back()) && times.back() <= 60,
            "trajectory times must start at 0 and last at most 60 s");
    Speed(trajectory.speed_scale, context);
    Tolerance(trajectory.tolerance_rad);
    for (std::size_t index = 0; index < positions.size(); ++index) {
        Target(positions[index], context);
        if (index == 0) continue;
        Require(std::isfinite(times[index]) && times[index] > times[index - 1],
                "trajectory times must increase");
        const auto ticks =
                std::max(1.0, (times[index] - times[index - 1]) * trajectory.command_rate_hz);
        for (std::size_t joint = 0; joint < positions[index].size(); ++joint) {
            Require(std::abs(positions[index][joint] - positions[index - 1][joint]) <=
                            trajectory.max_step_rad * ticks + 1e-6,
                    "trajectory step exceeds max_step_rad");
        }
    }
    // The first point must match live feedback; DeviceSession checks this before submission.
}

DeviceStep ParseHold(const YAML::Node& node) {
    Keys(node, {"capability"});
    return {node["capability"].as<std::string>(), {}, {}};
}
}  // namespace

void RegisterMotionCapabilities(CapabilityRegistry* registry) {
    registry->Register(
            {"joint_point", "Move to a joint target", ParsePoint, PointSupport, ValidatePoint,
                    [](DeviceSession& session, const DeviceStep& step,
                            const std::function<bool()>& cancelled) {
                        return session.MoveArm(step.arm, cancelled);
                    }});
    registry->Register({"effector_position", "Move end-effector channels", ParseEffector,
            EffectorSupport, ValidateEffectorPoint,
            [](DeviceSession& session, const DeviceStep& step,
                    const std::function<bool()>& cancelled) {
                return session.MoveEffector(step.effector, cancelled);
            }});
    registry->Register({"joint_jog", "Move one joint relative to its measured position", ParseJog,
            PointSupport, ValidateJog,
            [](DeviceSession& session, const DeviceStep& step,
                    const std::function<bool()>& cancelled) {
                const auto& jog = Parameters<JointJog>(step);
                return session.JogArm(jog.joint_index, jog.delta_rad, jog.speed_scale, cancelled);
            }});
    registry->Register({"joint_path", "Move through continuous joint waypoints", ParsePath,
            [](const CapabilityContext& context) {
                return context.arm.joint_path ? "" : "arm driver has no joint path";
            },
            ValidatePath,
            [](DeviceSession& session, const DeviceStep& step,
                    const std::function<bool()>& cancelled) {
                return session.MoveArmPath(Parameters<JointPath>(step), cancelled);
            }});
    registry->Register({"joint_trajectory", "Execute a timed joint trajectory", ParseTrajectory,
            [](const CapabilityContext& context) {
                return context.arm.joint_trajectory ? "" : "arm driver has no joint trajectory";
            },
            ValidateTrajectory,
            [](DeviceSession& session, const DeviceStep& step,
                    const std::function<bool()>& cancelled) {
                return session.MoveArmTrajectory(Parameters<JointTrajectory>(step), cancelled);
            }});
    registry->Register({"arm_hold", "Hold the arm and verify the stop", ParseHold,
            [](const CapabilityContext& context) {
                return context.arm.verified_hold ? "" : "arm stop cannot be verified";
            },
            [](const DeviceStep&, const CapabilityContext&) {},
            [](DeviceSession& session, const DeviceStep&, const std::function<bool()>&) {
                return session.HoldArm();
            }});
    registry->Register({"effector_hold", "Hold the end effector and verify the stop", ParseHold,
            [](const CapabilityContext& context) {
                if (!context.effector) return "device has no end effector";
                if (context.effector->verified_hold) return "";
                return "end-effector stop cannot be verified";
            },
            [](const DeviceStep&, const CapabilityContext&) {},
            [](DeviceSession& session, const DeviceStep&, const std::function<bool()>&) {
                return session.HoldEffector();
            }});
}
}  // namespace spacemit::manipulation
