// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Private model contract and transport-independent role session; no engine dependencies.
#ifndef SESSION_BACKEND_H
#define SESSION_BACKEND_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/device.h"

namespace spacemit::manipulation::simulation {
struct ArmMapping {
    std::string joint;
    std::string actuator;
    double max_velocity_rad_s = 0;
};

struct EffectorMapping {
    std::string channel;
    std::string joint;
    std::string actuator;
    double joint_closed = 0;
    double joint_open = 0;
    double control_closed = 0;
    double control_open = 0;
};

struct Mapping {
    std::string filename;
    std::string scene;
    std::string keyframe;
    std::vector<ArmMapping> arm;
    std::vector<EffectorMapping> effector;
    double arm_moving_threshold_rad_s = 0.002;
    double effector_moving_threshold = 0.002;
    double hold_timeout_s = 2;
    bool synchronized_joint_point = false;
};

struct TransportOptions {
    bool remote = false;
    std::string host = "127.0.0.1";
    unsigned port = 9100;
};

Mapping LoadMapping(const DriverOptions& options);
TransportOptions ParseTransport(const DriverOptions& options);
void ValidateArmMapping(const DeviceConfig& config, const Mapping& mapping);
void ValidateEffectorMapping(const EffectorConfig& config, const Mapping& mapping);

// Methods retain the public Arm/EndEffector exception and fresh-feedback contracts.
class SessionBackend {
public:
    virtual ~SessionBackend() = default;
    virtual void Acquire() = 0;
    virtual void Release() noexcept = 0;
    virtual ArmState ReadArm() = 0;
    virtual EffectorState ReadEffector() = 0;
    virtual void SubmitArm(const JointPoint& point) = 0;
    virtual void SubmitEffector(const EffectorPoint& point) = 0;
    virtual bool Hold(bool arm) = 0;
};

Devices WrapSession(std::shared_ptr<SessionBackend> session, bool arm, bool effector);

// Stable, path-free JSON contract. Neither peer may request remote scene/path loading.
std::string SessionContract(const Mapping& mapping, const std::optional<DeviceConfig>& arm,
        const std::optional<EffectorConfig>& effector);
}  // namespace spacemit::manipulation::simulation
#endif  // SESSION_BACKEND_H
