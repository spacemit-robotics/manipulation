// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file types.h
 * @brief Value types shared by device contracts, drivers and configuration.
 *
 * Units are radians and seconds unless a name says otherwise.
 */
#ifndef MANIPULATION_TYPES_H_
#define MANIPULATION_TYPES_H_

#include <chrono>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace spacemit::manipulation {

/// Monotonic clock for feedback timestamps and command deadlines.
using Clock = std::chrono::steady_clock;
/// Driver-specific settings: the `options` map of a device configuration.
using DriverOptions = std::map<std::string, std::string>;

/**
 * @brief Outcome of a device command.
 *
 * kTimedOut, kFault and kStopUnknown latch the session until it is reconnected.
 */
enum class Result {
    /// No command has run yet.
    kIdle,
    /// The command is executing.
    kRunning,
    /// The completion condition of the command was met.
    kSucceeded,
    /// Cancelled, and the driver confirmed the stop.
    kCancelled,
    /// The deadline passed, and the driver confirmed the stop.
    kTimedOut,
    /// Feedback, controller or write failure; the device was stopped.
    kFault,
    /// The device could not be confirmed to have stopped.
    kStopUnknown,
    /// Delivered and its execution time elapsed, but completion was not measured (an end effector
    /// without position feedback). It is neither success nor failure.
    kUnverified
};

/// Configured position limits of one joint.
struct JointLimit {
    /// Joint name, as the driver and the robot description use it.
    std::string name;
    /// Lower position limit, rad.
    double minimum_rad;
    /// Upper position limit, rad.
    double maximum_rad;
};

/// Arm section of a device configuration.
struct DeviceConfig {
    /// Name of the arm in logs and results.
    std::string device_id;
    /// Physical resource; one writer per resource_id on this host.
    std::string resource_id;
    /// Joints in target order, 1 to 16.
    std::vector<JointLimit> joints;
    /// Arrival tolerance, (0, 0.1] rad.
    double tolerance_rad = 0.01;
    /// Deadline of one command, (0, 300] s.
    double timeout_s = 5.0;
    /// Oldest feedback sample accepted as current, (0, 5] s.
    double maximum_age_s = 0.2;
};

/// Result of one device command.
struct CommandResult {
    /// Identifier of the command within its session.
    std::uint64_t command_id = 0;
    /// Outcome of the command.
    Result status = Result::kIdle;
    /// Evidence for the outcome, starting with a stable keyword such as `target_reached`.
    std::string detail;
};

/**
 * @brief Thrown when a request is refused before anything was written to the device.
 *
 * Session state is unchanged and no stop is required; any other exception from a write may mean
 * partial execution.
 */
class CommandRejected : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief Check an arm configuration offline.
 *
 * @param config Arm configuration.
 * @throws std::runtime_error If a field is missing or outside its documented range.
 */
void ValidateDevice(const DeviceConfig& config);

/**
 * @brief Name of a result for logs and JSON output, such as "succeeded".
 *
 * @param result Result to name.
 * @return Static string; never null.
 */
const char* ResultName(Result result);

/**
 * @brief Host-wide lease directory shared by every tool on this host.
 *
 * @return $MANIPULATION_LOCK_DIRECTORY when set to an absolute path, otherwise the build-time
 *         MANIPULATION_LOCK_DIRECTORY. Independent of the working directory.
 */
const char* DefaultLockDirectory();

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_TYPES_H_
