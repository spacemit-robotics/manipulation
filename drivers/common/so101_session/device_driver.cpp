// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Registry-local composition; the arm and optional gripper share one serial session.
#include "so101_session.h"

#include <set>
#include <stdexcept>

namespace spacemit::manipulation {
namespace {
void CheckKeys(const DriverOptions& options, const std::set<std::string>& allowed) {
    for (const auto& item : options) {
        if (!allowed.count(item.first)) {
            throw std::runtime_error("unsupported SO101 role option: " + item.first);
        }
    }
}

so101::Settings ArmSettings(const DeviceConfig& config, const DriverOptions& options) {
    CheckKeys(options, {"port", "arm_ids", "arm_calibration", "max_velocity_rad_s"});
    DeviceDefinition definition;
    definition.arm = config;
    return so101::Parse(definition, options);
}

so101::Settings EffectorSettings(const EffectorConfig& config, const DriverOptions& options) {
    CheckKeys(options,
            {"port", "gripper_id", "gripper_calibration", "grip_load_threshold",
                    "max_velocity_rad_s"});
    DeviceDefinition definition;
    definition.effector = config;
    return so101::Parse(definition, options);
}

DriverOptions SharedOptions(
        const DriverOptions& arm_options, const DriverOptions& effector_options) {
    DriverOptions result = arm_options;
    for (const auto& item : effector_options) {
        const auto inserted = result.emplace(item);
        if (!inserted.second && inserted.first->second != item.second) {
            throw std::runtime_error("conflicting shared SO101 option: " + item.first);
        }
    }
    return result;
}
}  // namespace

void RegisterSo101Device(DeviceRegistry* registry) {
    registry->Register({"so101", 1,
            [](const DeviceDefinition& definition, const DriverOptions& options) {
                so101::Parse(definition, options);
            },
            [](const DeviceDefinition& definition, const DriverOptions& options) {
                auto session =
                        std::make_shared<so101::SharedSession>(so101::Parse(definition, options));
                Devices devices;
                devices.arm = so101::MakeArm(definition.arm, session);
                if (!definition.effector.channels.empty()) {
                    devices.effector = so101::MakeGripper(definition.effector, session);
                }
                return devices;
            },
            0, 120, so101::Resolve});
    registry->RegisterArm({"so101", 1,
            [](const DeviceConfig& config, const DriverOptions& options) {
                ArmSettings(config, options);
            },
            [](const DeviceConfig& config, const DriverOptions& options) {
                return so101::MakeArm(config,
                        std::make_shared<so101::SharedSession>(ArmSettings(config, options)));
            },
            so101::Resolve});
    registry->RegisterEffector({"so101_gripper", 1,
            [](const EffectorConfig& config, const DriverOptions& options) {
                EffectorSettings(config, options);
            },
            [](const EffectorConfig& config, const DriverOptions& options) {
                return so101::MakeGripper(config,
                        std::make_shared<so101::SharedSession>(EffectorSettings(config, options)));
            },
            so101::Resolve});
    registry->RegisterShared({"so101", "so101_gripper",
            [](const DeviceDefinition& definition, const DriverOptions& arm_options,
                    const DriverOptions& effector_options) {
                so101::Parse(definition, SharedOptions(arm_options, effector_options));
            },
            [](const DeviceDefinition& definition, const DriverOptions& arm_options,
                    const DriverOptions& effector_options) {
                auto session = std::make_shared<so101::SharedSession>(
                        so101::Parse(definition, SharedOptions(arm_options, effector_options)));
                Devices devices;
                devices.arm = so101::MakeArm(definition.arm, session);
                devices.effector = so101::MakeGripper(definition.effector, session);
                return devices;
            }});
}
}  // namespace spacemit::manipulation
