// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Private shared physics session. No MuJoCo C types escape the engine component.
#ifndef MUJOCO_SESSION_H
#define MUJOCO_SESSION_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_world.h"
#include "session_backend.h"

namespace spacemit::manipulation::simulation {
class SharedSession final : public SessionBackend {
public:
    SharedSession(Mapping mapping, std::optional<DeviceConfig> arm,
            std::optional<EffectorConfig> effector);
    ~SharedSession() override;
    void Acquire() override;
    void Release() noexcept override;
    ArmState ReadArm() override;
    EffectorState ReadEffector() override;
    void SubmitArm(const JointPoint& point) override;
    void SubmitEffector(const EffectorPoint& point) override;
    bool Hold(bool arm) override;
    // Freeze all configured roles in one control batch before verifying either role is still.
    bool HoldAll();
    void Reset();
    mujoco_sim::WorldSnapshot Snapshot();
    void AttachWeld(const std::string& equality, const std::string& body1, const std::string& body2,
            const std::string& left_pad, const std::string& right_pad);
    void ReleaseWeld(const std::string& equality);

private:
    struct Motion {
        std::vector<double> reference;
        std::vector<double> target;
        std::vector<double> rates;
        bool initialized = false;
        bool active = false;
    };

    void ValidateModel();
    void Sample();
    void CheckReady() const;
    void Advance(Motion* motion, bool arm, double timestep);
    void Run() noexcept;
    void Stop() noexcept;

    Mapping mapping_;
    std::optional<DeviceConfig> arm_config_;
    std::optional<EffectorConfig> effector_config_;
    std::unique_ptr<mujoco_sim::World> world_;
    std::mutex lifecycle_mutex_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::thread thread_;
    unsigned owners_ = 0;
    std::uint64_t sample_sequence_ = 0;
    bool stopping_ = false;
    std::string fault_;
    ArmState arm_sample_;
    EffectorState effector_sample_;
    Motion arm_motion_;
    Motion effector_motion_;
};
}  // namespace spacemit::manipulation::simulation
#endif  // MUJOCO_SESSION_H
