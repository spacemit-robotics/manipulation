// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Optional local simulation facade; construction is offline and Start owns one persistent world.
#ifndef MUJOCO_RUNTIME_H
#define MUJOCO_RUNTIME_H

#include <memory>
#include <string>

#include "manipulation/device.h"

namespace mujoco_sim {
class WorldSnapshot;
}

namespace spacemit::manipulation {
class MujocoRuntime {
public:
    MujocoRuntime(const DeviceDefinition& definition, const DriverOptions& options);
    ~MujocoRuntime();
    MujocoRuntime(const MujocoRuntime&) = delete;
    MujocoRuntime& operator=(const MujocoRuntime&) = delete;
    // Explicit world owner reference; repeated Start is rejected, never resets an active world.
    void Start();
    void Stop() noexcept;
    // Explicitly restore the configured keyframe, only while no control role is connected.
    void Reset();
    // Freeze both roles atomically, then verify three fresh still samples per role within the
    // mapping hold timeout. Returns false on unverified stop; caller must exclude new commands.
    bool Hold();
    // Offline wrappers over the same session. DeviceSession still owns the control resource lease.
    Devices CreateDevices();
    // Opaque owning snapshot, safe after a later physics step or runtime stop.
    mujoco_sim::WorldSnapshot Snapshot();
    // Check both pad bodies contact body2 and attach at the measured pose under one world lock.
    void AttachWeld(const std::string& equality, const std::string& body1, const std::string& body2,
            const std::string& left_pad, const std::string& right_pad);
    void ReleaseWeld(const std::string& equality);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace spacemit::manipulation
#endif  // MUJOCO_RUNTIME_H
