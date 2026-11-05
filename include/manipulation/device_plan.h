// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef MANIPULATION_DEVICE_PLAN_H_
#define MANIPULATION_DEVICE_PLAN_H_
#include <string>
#include <vector>
#include "manipulation/device.h"

namespace spacemit::manipulation {
struct DeviceStep {
    std::string capability;
    JointPoint arm;
    EffectorPoint effector;
};

struct DevicePlan {
    DeviceDefinition definition;
    std::string driver;
    std::string model;  // Optional description id under descriptions/robots/<model>/.
    DriverOptions options;
    std::vector<DeviceStep> steps;
};

DevicePlan LoadDevicePlan(const std::string& filename, const DeviceRegistry& registry);
CommandResult RunDeviceSteps(DeviceSession& session, const std::vector<DeviceStep>& steps,
                             const std::function<bool()>& cancelled = {});
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_DEVICE_PLAN_H_
