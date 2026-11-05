// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Shared value validation and names.
#include "manipulation/types.h"

#include <cmath>
#include <cstdlib>
#include <set>

#ifndef MANIPULATION_LOCK_DIRECTORY
#define MANIPULATION_LOCK_DIRECTORY "/run/lock/spacemit-manipulation"
#endif

namespace spacemit::manipulation {
namespace {
bool ValidId(const std::string& name) {
    return !name.empty() && name.size() <= 80 &&
           name.find_first_not_of(
                   "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
                   std::string::npos;
}

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

void ValidateDevice(const DeviceConfig& config) {
    Require(ValidId(config.device_id) && ValidId(config.resource_id), "invalid device/resource id");
    Require(!config.joints.empty() && config.joints.size() <= 16, "joint count must be 1..16");
    std::set<std::string> names;
    for (const auto& joint : config.joints) {
        Require(ValidId(joint.name) && names.insert(joint.name).second,
                "invalid/duplicate joint name");
        Require(std::isfinite(joint.minimum_rad) && std::isfinite(joint.maximum_rad) &&
                        joint.minimum_rad < joint.maximum_rad,
                "invalid joint limits");
    }
    Require(std::isfinite(config.tolerance_rad) && config.tolerance_rad > 0 &&
                    config.tolerance_rad <= 0.1,
            "invalid tolerance_rad");
    Require(std::isfinite(config.timeout_s) && config.timeout_s > 0 && config.timeout_s <= 300,
            "invalid timeout_s");
    Require(std::isfinite(config.maximum_age_s) && config.maximum_age_s > 0 &&
                    config.maximum_age_s <= 5,
            "invalid maximum_age_s");
}

const char* ResultName(Result result) {
    switch (result) {
        case Result::kIdle:
            return "idle";
        case Result::kRunning:
            return "running";
        case Result::kSucceeded:
            return "succeeded";
        case Result::kCancelled:
            return "cancelled";
        case Result::kTimedOut:
            return "timed_out";
        case Result::kFault:
            return "fault";
        case Result::kStopUnknown:
            return "stop_unknown";
        case Result::kUnverified:
            return "unverified";
    }
    return "unknown";
}

const char* DefaultLockDirectory() {
    // An explicit absolute override (for isolated tests or a service RuntimeDirectory) wins;
    // every cooperating program must then use the same value for leases to exclude each other.
    const char* overridden = std::getenv("MANIPULATION_LOCK_DIRECTORY");
    return overridden && overridden[0] == '/' ? overridden : MANIPULATION_LOCK_DIRECTORY;
}

}  // namespace spacemit::manipulation
