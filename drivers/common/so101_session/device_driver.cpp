// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Registry-local composition; the arm and optional gripper share one serial session.
#include "so101_session.h"

namespace spacemit::manipulation {
void RegisterSo101Device(DeviceRegistry* registry) {
    registry->Register({"so101", 1,
        [](const DeviceDefinition& definition, const DriverOptions& options) {
            so101::Parse(definition, options);
        },
        [](const DeviceDefinition& definition, const DriverOptions& options) {
            auto session = std::make_shared<so101::SharedSession>(so101::Parse(definition, options));
            Devices devices;
            devices.arm = so101::MakeArm(definition.arm, session);
            if (!definition.effector.channels.empty()) {
                devices.effector = so101::MakeGripper(definition.effector, session);
            }
            return devices;
        }, 0, 120, so101::Resolve});
}
}  // namespace spacemit::manipulation
