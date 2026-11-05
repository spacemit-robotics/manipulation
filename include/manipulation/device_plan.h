// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file device_plan.h
 * @brief Device configuration files (device.yaml) and their ordered point steps.
 */
#ifndef MANIPULATION_DEVICE_PLAN_H_
#define MANIPULATION_DEVICE_PLAN_H_
#include <string>
#include <vector>
#include "manipulation/device.h"

namespace spacemit::manipulation {
/// One entry of `steps`: an arm point or an end-effector move.
struct DeviceStep {
    /// "joint_point" or "effector_position".
    std::string capability;
    /// Arm target, used when capability is "joint_point".
    JointPoint arm;
    /// End-effector target, used when capability is "effector_position".
    EffectorPoint effector;
};

/// A parsed device configuration.
struct DevicePlan {
    /// Arm and end-effector configuration.
    DeviceDefinition definition;
    /// Registered driver name (`device.driver`).
    std::string driver;
    /// Optional robot description under descriptions/robots/<model>/.
    std::string model;
    /// Driver options, with declared paths resolved against the configuration directory.
    DriverOptions options;
    /// Ordered steps; empty when the configuration declares none.
    std::vector<DeviceStep> steps;
};

/**
 * @brief Load and validate a device configuration offline.
 *
 * @param filename Single-document YAML file of at most 64 KiB with schema_version 3. Relative
 *        option paths resolve against the file's real directory.
 * @param registry Registry that must contain the configured driver.
 * @return The validated plan; no device is opened.
 * @throws std::runtime_error On unknown or duplicate keys, invalid values or an unknown driver.
 */
DevicePlan LoadDevicePlan(const std::string& filename, const DeviceRegistry& registry);

/**
 * @brief Execute steps in order on a connected session.
 *
 * Stops at the first step that neither succeeds nor is unverified.
 *
 * @param session Connected session of the same device.
 * @param steps Steps to execute.
 * @param cancelled Optional cancellation callback, polled every 10 ms.
 * @return Result of the failing step, otherwise kSucceeded, or kUnverified when any step was
 *         unverified; kIdle for an empty list.
 */
CommandResult RunDeviceSteps(DeviceSession& session, const std::vector<DeviceStep>& steps,
                             const std::function<bool()>& cancelled = {});
}  // namespace spacemit::manipulation
#endif  // MANIPULATION_DEVICE_PLAN_H_
