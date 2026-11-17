// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Canonical typed contract, independent of host paths and user-selected resource aliases.
#include "session_backend.h"

#include "wire_protocol.h"

namespace spacemit::manipulation::simulation {
std::string SessionContract(const Mapping& mapping, const std::optional<DeviceConfig>& arm,
        const std::optional<EffectorConfig>& effector) {
    namespace w = wire;
    std::string arms = "[", tools = "[", limits = "[";
    for (const auto& item : mapping.arm) {
        if (arms.size() > 1) arms += ',';
        arms += w::Object({{"joint", w::Quote(item.joint)}, {"actuator", w::Quote(item.actuator)},
                {"velocity", w::Number(item.max_velocity_rad_s)}});
    }
    for (const auto& item : mapping.effector) {
        if (tools.size() > 1) tools += ',';
        tools += w::Object({{"channel", w::Quote(item.channel)}, {"joint", w::Quote(item.joint)},
                {"actuator", w::Quote(item.actuator)},
                {"joint_closed", w::Number(item.joint_closed)},
                {"joint_open", w::Number(item.joint_open)},
                {"control_closed", w::Number(item.control_closed)},
                {"control_open", w::Number(item.control_open)}});
    }
    if (arm) {
        for (const auto& joint : arm->joints) {
            if (limits.size() > 1) limits += ',';
            limits += w::Object(
                    {{"name", w::Quote(joint.name)}, {"minimum", w::Number(joint.minimum_rad)},
                            {"maximum", w::Number(joint.maximum_rad)}});
        }
    }
    std::string arm_contract = "null", effector_contract = "null";
    // Freshness budgets belong to each receiving endpoint, not the physical command contract.
    // The server bounds local sampling age; the client separately bounds source age plus RTT.
    // Omitting them here never changes either endpoint's age validation or motion limits.
    if (arm) {
        arm_contract =
                w::Object({{"limits", limits + ']'}, {"tolerance", w::Number(arm->tolerance_rad)},
                        {"timeout", w::Number(arm->timeout_s)}});
    }
    if (effector) {
        effector_contract = w::Object({{"channels", w::Strings(effector->channels)},
                {"tolerance", w::Number(effector->tolerance)},
                {"timeout", w::Number(effector->timeout_s)}});
    }
    return w::Object({{"version", "1"}, {"keyframe", w::Quote(mapping.keyframe)},
            {"arm_mapping", arms + ']'}, {"effector_mapping", tools + ']'}, {"arm", arm_contract},
            {"effector", effector_contract},
            {"arm_moving", w::Number(mapping.arm_moving_threshold_rad_s)},
            {"effector_moving", w::Number(mapping.effector_moving_threshold)},
            {"hold_timeout", w::Number(mapping.hold_timeout_s)},
            {"synchronized_joint_point", mapping.synchronized_joint_point ? "true" : "false"}});
}
}  // namespace spacemit::manipulation::simulation
