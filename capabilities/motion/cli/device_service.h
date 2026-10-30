// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef DEVICE_SERVICE_H
#define DEVICE_SERVICE_H
#include "manipulation/device.h"
// Quoted, escaped JSON string literal.
std::string JsonString(const std::string& text);
int ServeDevice(spacemit::manipulation::DeviceSession& session, bool allow_motion, bool allow_enable,
    const std::function<bool()>& interrupted, const std::string& driver);
#endif  // DEVICE_SERVICE_H
