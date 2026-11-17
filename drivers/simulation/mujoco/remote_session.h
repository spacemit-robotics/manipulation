// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Remote roles share a fenced persistent controller connection and an independent heartbeat.
#ifndef REMOTE_SESSION_H
#define REMOTE_SESSION_H

#include <atomic>
#include <mutex>
#include <thread>

#include "wire_protocol.h"

namespace spacemit::manipulation::simulation {
class RemoteSession final : public SessionBackend {
public:
    RemoteSession(Mapping mapping, std::optional<DeviceConfig> arm,
            std::optional<EffectorConfig> effector, TransportOptions transport);
    ~RemoteSession() override;
    void Acquire() override;
    void Release() noexcept override;
    ArmState ReadArm() override;
    EffectorState ReadEffector() override;
    void SubmitArm(const JointPoint& point) override;
    void SubmitEffector(const EffectorPoint& point) override;
    bool Hold(bool arm) override;

private:
    YAML::Node Call(const std::string& op, const std::string& fields = "", double timeout_s = 0.5);
    void Refresh();
    void Heartbeat() noexcept;
    void Stop() noexcept;
    void Ready() const;
    Mapping mapping_;
    std::optional<DeviceConfig> arm_;
    std::optional<EffectorConfig> effector_;
    TransportOptions transport_;
    std::mutex lifecycle_;
    std::mutex mutex_;
    std::thread heartbeat_;
    std::atomic<bool> stopped_{true};
    std::atomic<bool> failed_{false};
    int socket_ = -1;
    int heartbeat_socket_ = -1;
    unsigned owners_ = 0;
    std::string epoch_;
    std::string token_;
    std::uint64_t fence_ = 0;
    std::uint64_t request_ = 0;
    std::uint64_t arm_required_ = 0;
    std::uint64_t effector_required_ = 0;
    std::uint64_t arm_source_ = 0;
    std::uint64_t effector_source_ = 0;
    std::uint64_t arm_sequence_ = 0;
    std::uint64_t effector_sequence_ = 0;
    double arm_remote_age_ = 0;
    double effector_remote_age_ = 0;
    ArmState arm_sample_;
    EffectorState effector_sample_;
};
}  // namespace spacemit::manipulation::simulation
#endif  // REMOTE_SESSION_H
