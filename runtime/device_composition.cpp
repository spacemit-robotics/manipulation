// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Offline independent role selection. No device connections or resource leases.
#include "manipulation/device_composition.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "manipulation/device.h"

namespace spacemit::manipulation {
namespace {
DriverOptions Options(const YAML::Node& node) {
    DriverOptions options;
    if (!node) return options;
    if (!node.IsMap() || node.size() > 32) throw std::runtime_error("invalid driver options");
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        const auto value = item.second.as<std::string>();
        if (key.empty() || key.size() > 80 || value.size() > 4096 ||
                !options.emplace(key, value).second) {
            throw std::runtime_error("invalid/duplicate driver option");
        }
    }
    return options;
}
}  // namespace

DeviceComposition ParseDeviceComposition(const YAML::Node& device, DeviceDefinition* definition,
        const DeviceRegistry& registry, const std::string& directory) {
    if (!definition || device["driver"] || device["options"]) {
        throw std::runtime_error("cannot mix whole-device and independent driver selection");
    }
    DeviceComposition result;
    result.arm_driver = device["arm"]["driver"].as<std::string>();
    result.arm_options = Options(device["arm"]["options"]);
    const auto& arm = registry.GetArm(result.arm_driver);
    if (arm.resolve_options)
        result.arm_options = arm.resolve_options(result.arm_options, directory);
    if (device["effector"]) {
        result.effector_driver = device["effector"]["driver"].as<std::string>();
        result.effector_options = Options(device["effector"]["options"]);
        const auto& effector = registry.GetEffector(result.effector_driver);
        if (effector.resolve_options) {
            result.effector_options = effector.resolve_options(result.effector_options, directory);
        }
    }
    registry.ValidateComposition(*definition, result);
    return result;
}
}  // namespace spacemit::manipulation
