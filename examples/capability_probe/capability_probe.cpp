// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Example separately registered task capability using only public device feedback.
#include "manipulation/capability.h"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "manipulation/device_plan.h"

namespace spacemit::manipulation {
namespace {
struct PositionCheck {
    std::vector<double> target_rad;
    double tolerance_rad = 0.01;
};
}  // namespace

void RegisterProbeCapability(CapabilityRegistry* registry) {
    registry->Register({"assert_joint_position", "Check measured joints without sending a target",
            [](const YAML::Node& node) {
                for (const auto& item : node) {
                    const auto key = item.first.as<std::string>();
                    if (key != "capability" && key != "target_rad" && key != "tolerance_rad")
                        throw std::runtime_error("unknown position check parameter: " + key);
                }
                PositionCheck check;
                check.target_rad = node["target_rad"].as<std::vector<double>>();
                if (node["tolerance_rad"]) check.tolerance_rad = node["tolerance_rad"].as<double>();
                return DeviceStep{"assert_joint_position", {}, {}, std::move(check)};
            },
            [](const CapabilityContext&) { return ""; },
            [](const DeviceStep& step, const CapabilityContext& context) {
                const auto& check = std::any_cast<const PositionCheck&>(step.parameters);
                if (check.target_rad.size() != context.definition.arm.joints.size() ||
                        !std::isfinite(check.tolerance_rad) || check.tolerance_rad <= 0 ||
                        check.tolerance_rad > 0.1)
                    throw std::runtime_error("invalid joint position check");
                for (double position : check.target_rad) {
                    if (!std::isfinite(position))
                        throw std::runtime_error("joint position must be finite");
                }
            },
            [](DeviceSession& session, const DeviceStep& step, const std::function<bool()>&) {
                const auto& check = std::any_cast<const PositionCheck&>(step.parameters);
                const auto state = session.ReadArm();
                for (std::size_t index = 0; index < check.target_rad.size(); ++index) {
                    if (std::abs(state.joints_rad[index] - check.target_rad[index]) >
                            check.tolerance_rad)
                        throw CommandRejected("joint position check did not match feedback");
                }
                return CommandResult{0, Result::kSucceeded, "joint_position_checked"};
            }});
}
}  // namespace spacemit::manipulation
