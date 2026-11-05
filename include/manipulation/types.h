// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Value types shared by device contracts, drivers and configuration. Units: rad, s.
#ifndef MANIPULATION_TYPES_H_
#define MANIPULATION_TYPES_H_

#include <chrono>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace spacemit::manipulation {

using Clock = std::chrono::steady_clock;
using DriverOptions = std::map<std::string, std::string>;

// kUnverified: command delivered and its execution time elapsed, but completion was not measured
// (end effector configured without position feedback). It is neither success nor failure.
enum class Result {
    kIdle,
    kRunning,
    kSucceeded,
    kCancelled,
    kTimedOut,
    kFault,
    kStopUnknown,
    kUnverified
};

struct JointLimit {
    std::string name;
    double minimum_rad;
    double maximum_rad;
};

struct DeviceConfig {
    std::string device_id;
    std::string resource_id;  // Physical resource; one writer per resource_id on this host.
    std::vector<JointLimit> joints;
    double tolerance_rad = 0.01;
    double timeout_s = 5.0;
    double maximum_age_s = 0.2;
};

struct CommandResult {
    std::uint64_t command_id = 0;
    Result status = Result::kIdle;
    std::string detail;
};

// Thrown when a request is refused before anything was written to the device.
// Session state is unchanged and no stop is required; any other exception from a
// write may mean partial execution.
class CommandRejected : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

void ValidateDevice(const DeviceConfig& config);
const char* ResultName(Result result);

// Host-wide lease directory: $MANIPULATION_LOCK_DIRECTORY when set to an absolute path,
// otherwise the build-time MANIPULATION_LOCK_DIRECTORY. Independent of the working directory.
const char* DefaultLockDirectory();

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_TYPES_H_
