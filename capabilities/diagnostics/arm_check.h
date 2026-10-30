// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Supervised bounded arm check over the public device session.
#ifndef ARM_CHECK_H
#define ARM_CHECK_H

#include <functional>
#include <ostream>

#include "manipulation/device_plan.h"

namespace spacemit::manipulation {

// Joint 1 moves 0.01 rad at 1% and back, then cancel, return and reconnect are checked.
// timeout_only instead checks a 1 ms deadline, its latch and a return after reconnect.
// Requires an already enabled, stationary, normal arm; never enables, homes or clears faults.
// Returns 0 when every step succeeded, 1 when a step failed or was interrupted.
int RunArmCheck(const DevicePlan& plan, const DeviceRegistry& registry, bool timeout_only,
    const std::function<bool()>& interrupted, std::ostream& out);

}  // namespace spacemit::manipulation
#endif  // ARM_CHECK_H
