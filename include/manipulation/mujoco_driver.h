// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file mujoco_driver.h
 * @brief Identify the model-specific, mapped and compatible MuJoCo role names.
 */
#ifndef MUJOCO_DRIVER_H
#define MUJOCO_DRIVER_H

#include <string>

namespace spacemit::manipulation {
/// Match a supported MuJoCo arm role name, without opening a device or loading a model.
/// Link spacemit::manipulation_drivers built with local or remote MuJoCo support.
bool IsMujocoArmDriver(const std::string& name);

/// Match a supported MuJoCo effector role name, without opening a device or loading a model.
/// Link spacemit::manipulation_drivers built with local or remote MuJoCo support.
bool IsMujocoEffectorDriver(const std::string& name);
}  // namespace spacemit::manipulation
#endif  // MUJOCO_DRIVER_H
