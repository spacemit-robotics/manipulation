// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file device_composition.h
 * @brief Independent arm and end-effector selection for configured device assemblies.
 */
#ifndef DEVICE_COMPOSITION_H
#define DEVICE_COMPOSITION_H

#include <map>
#include <string>

namespace YAML {
class Node;
}

namespace spacemit::manipulation {
class DeviceRegistry;
struct DeviceDefinition;

/// Driver-specific selections. Empty arm_driver denotes a legacy whole-device selection.
struct DeviceComposition {
    std::string arm_driver;
    std::string effector_driver;
    std::map<std::string, std::string> arm_options;
    std::map<std::string, std::string> effector_options;
};

/**
 * @brief Parse, resolve and validate role selections from a device mapping, offline.
 *
 * Common arm/effector fields must already have been parsed into definition. Options are resolved
 * relative to directory. Mixed legacy and independent selections are rejected.
 */
DeviceComposition ParseDeviceComposition(const YAML::Node& device, DeviceDefinition* definition,
        const DeviceRegistry& registry, const std::string& directory);
}  // namespace spacemit::manipulation
#endif  // DEVICE_COMPOSITION_H
