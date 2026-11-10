// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Composition is vendor-local; core never constructs model-specific devices.
#include "agx_session.h"
#include <stdexcept>

namespace spacemit::manipulation {
namespace {
void Validate(const DeviceDefinition& definition, const DriverOptions& options) {
    agx::ValidateArm(definition.arm, options);
    const std::vector<std::string> channels{"thumb_tip", "thumb_base", "index_finger",
            "middle_finger", "ring_finger", "pinky_finger"};
    if (!definition.effector.channels.empty() && definition.effector.channels != channels) {
        throw std::runtime_error("Revo2 requires six named channels in documented order");
    }
}

void ValidateEffectorDriver(const EffectorConfig& config, const DriverOptions& options) {
    agx::ValidateConnection(config.resource_id, options);
    const std::vector<std::string> channels{"thumb_tip", "thumb_base", "index_finger",
            "middle_finger", "ring_finger", "pinky_finger"};
    if (config.channels != channels) {
        throw std::runtime_error("Revo2 requires six named channels in documented order");
    }
}

void ValidateShared(const DeviceDefinition&, const DriverOptions& arm_options,
        const DriverOptions& effector_options) {
    if (arm_options != effector_options) {
        throw std::runtime_error("shared AGX connection requires identical transport options");
    }
}
}  // namespace

void RegisterAgxDevice(DeviceRegistry* registry) {
    registry->Register({"agx_nero_revo2", 1, Validate,
            [](const DeviceDefinition& definition, const DriverOptions& options) {
                auto connection = std::make_shared<agx::SharedSession>(options);
                Devices devices;
                devices.arm = agx::MakeArm(definition.arm, options, connection);
                if (!definition.effector.channels.empty()) {
                    devices.effector = agx::MakeEffector(definition.effector, connection);
                }
                return devices;
            },
            0.01, 2.55, agx::ResolveOptions});
    registry->RegisterArm({"nero", 1, agx::ValidateArm,
            [](const DeviceConfig& config, const DriverOptions& options) {
                return agx::MakeArm(config, options, std::make_shared<agx::SharedSession>(options));
            },
            agx::ResolveOptions});
    registry->RegisterEffector({"revo2", 1, ValidateEffectorDriver,
            [](const EffectorConfig& config, const DriverOptions& options) {
                return agx::MakeEffector(
                        config, std::make_shared<agx::SharedSession>(options, false));
            },
            agx::ResolveOptions});
    registry->RegisterShared({"nero", "revo2", ValidateShared,
            [](const DeviceDefinition& definition, const DriverOptions& arm_options,
                    const DriverOptions&) {
                auto connection = std::make_shared<agx::SharedSession>(arm_options);
                Devices devices;
                devices.arm = agx::MakeArm(definition.arm, arm_options, connection);
                devices.effector = agx::MakeEffector(definition.effector, connection);
                return devices;
            }});
}
}  // namespace spacemit::manipulation
