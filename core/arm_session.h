// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Internal arm execution used by DeviceSession. Not installed.
#ifndef MANIPULATION_CORE_ARM_SESSION_H_
#define MANIPULATION_CORE_ARM_SESSION_H_

#include <memory>
#include <string>

#include "manipulation/arm.h"

namespace spacemit::manipulation {

// Create the shared lease directory when missing and verify it cannot be replaced
// by another account (directory, not a symlink, owned by us/root or sticky).
void PrepareLockDirectory(const std::string& directory);

// Serialized, single-writer arm execution. Fault/timeout/stop_unknown latch until
// a deliberate disconnect/reconnect. A CommandRejected submission leaves state unchanged.
class ArmSession {
public:
    ArmSession(DeviceConfig config, std::unique_ptr<Arm> arm, std::string lock_directory);
    ~ArmSession();
    ArmSession(const ArmSession&) = delete;
    ArmSession& operator=(const ArmSession&) = delete;
    void Connect();
    void Disconnect() noexcept;
    ArmState Read();
    ArmState Diagnose();
    ArmCapabilities GetCapabilities() const;
    void Enable();
    void Disable();
    CommandResult Hold();
    CommandResult Submit(const JointPoint& point);
    // Sends the target and completes at once; the next target may start while the arm moves.
    CommandResult Follow(const JointPoint& point);
    CommandResult SubmitPath(const JointPath& path);  // Completion is judged at the last waypoint.
    // Likewise, at the last knot.
    CommandResult SubmitTrajectory(const JointTrajectory& trajectory);
    CommandResult Poll();
    CommandResult Cancel();
    CommandResult GetResult() const;
    // A command completed without waiting for arrival and the arm has not been seen still since.
    bool Settling() const;
    // The final target of the last command (the one the arm settles toward while Settling()).
    const std::vector<double>& LastTarget() const;

private:
    void Ready();
    void CheckTarget(const std::vector<double>& target_rad, double speed_scale);
    // Arrival: within tolerance and still. Delivered: the driver sent the last target. Within
    // tolerance: two fresh samples within tolerance, while the arm may still be settling.
    enum class Completion { kArrival, kDelivered, kWithinTolerance };
    template <typename Extra, typename Write>
    CommandResult Begin(const JointPoint& final_point, double tolerance_rad, Completion completion,
                        Extra extra_time_s, Write write);
    void Stop(Result result, const std::string& detail);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spacemit::manipulation
#endif  // MANIPULATION_CORE_ARM_SESSION_H_
