/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_native_snapshot.h
 * @brief Opt-in read-only native inspection of detached MuJoCo state.
 */
#ifndef MUJOCO_NATIVE_SNAPSHOT_H
#define MUJOCO_NATIVE_SNAPSHOT_H

#include <memory>

struct mjModel_;
struct mjData_;

namespace mujoco_sim {

class WorldSnapshot;

/**
 * Owns private model/data copies with refreshed position/contact geometry.
 * Never exposes World or immutable WorldSnapshot storage. Update reuses its
 * model copy for snapshots of the same World; weld edits are copied separately.
 * No control callback, integration or rendering is performed. Calls to MuJoCo
 * using these borrowed pointers must be read-only; do not retain the pointers
 * after destruction or Update. For IK, copy a separate mjData before modifying it.
 */
class NativeSnapshot {
public:
    explicit NativeSnapshot(const WorldSnapshot& snapshot);
    ~NativeSnapshot();
    NativeSnapshot(const NativeSnapshot&) = delete;
    NativeSnapshot& operator=(const NativeSnapshot&) = delete;
    /** Replace private state; model cloning occurs only when its source World changes. */
    void Update(const WorldSnapshot& snapshot);
    const mjModel_* Model() const;
    const mjData_* Data() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim

#endif  // MUJOCO_NATIVE_SNAPSHOT_H
