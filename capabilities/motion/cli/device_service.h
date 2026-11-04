// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef DEVICE_SERVICE_H
#define DEVICE_SERVICE_H
#include "manipulation/device.h"
// Quoted, escaped JSON string literal.
std::string JsonString(const std::string& text);
// Offline driver capabilities; no device connection or write.
std::string DeviceCapabilitiesJson(const spacemit::manipulation::DeviceSession& session);
int ServeDevice(spacemit::manipulation::DeviceSession& session, bool allow_motion, bool allow_enable,
    const std::function<bool()>& interrupted, const std::string& driver, bool allow_disable = false,
    bool allow_effector_enable = false, bool allow_effector_disable = false);
#endif  // DEVICE_SERVICE_H
