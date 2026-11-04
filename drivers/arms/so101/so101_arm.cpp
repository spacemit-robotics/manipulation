// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Calibrated SO101 arm, with explicit torque changes and measured holding stops.
#include "so101_session.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace spacemit::manipulation::so101 {
namespace {
class So101Arm final : public Arm {
public:
    So101Arm(DeviceConfig definition, std::shared_ptr<SharedSession> session)
        : definition_(std::move(definition)), session_(std::move(session)) {}
    ~So101Arm() override { Disconnect(); }
    ArmCapabilities GetCapabilities() const override {
        ArmCapabilities capabilities;
        capabilities.joint_point = true;
        capabilities.enable = true;
        capabilities.disable = true;
        capabilities.verified_hold = true;
        capabilities.minimum_speed_scale = kSpeedTickRadS / session_->Config().maximum_velocity;
        return capabilities;
    }
    void Connect() override {
        if (connected_) throw std::runtime_error("SO101 arm already connected");
        session_->Connect();
        connected_ = true;
        try {
            Read();
        } catch (...) {
            Disconnect();
            throw;
        }
    }
    void Disconnect() noexcept override {
        if (connected_) session_->Disconnect();
        connected_ = false;
    }
    ArmState Read() override {
        if (!connected_) throw std::runtime_error("SO101 arm disconnected");
        ArmState state;
        state.received_at = Clock::now();
        state.enabled = true;
        state.position_mode = true;
        const auto& settings = session_->Config();
        for (size_t i = 0; i < kArmJoints; ++i) {
            const auto sample = session_->Read(settings.ids[i]);
            if (settings.arm_calibrated) {
                const auto& calibration = settings.calibration[i];
                if (sample.offset != calibration.offset || sample.minimum > calibration.minimum ||
                    sample.maximum < calibration.maximum) {
                    throw std::runtime_error("SO101 motor " + std::to_string(settings.ids[i]) +
                        " calibration differs from the installed servo configuration");
                }
            }
            samples_[i] = sample;
            state.joints_rad.push_back(JointPosition(sample.position));
            state.enabled = state.enabled && sample.enabled;
            state.position_mode = state.position_mode && sample.position_mode;
            state.moving = state.moving || sample.moving;
            state.fault = state.fault || sample.fault;
            if (!state.firmware.empty()) state.firmware += ",";
            state.firmware += std::to_string(settings.ids[i]) + ":" + sample.firmware;
            const int minimum = settings.arm_calibrated ? settings.calibration[i].minimum : sample.minimum;
            const int maximum = settings.arm_calibrated ? settings.calibration[i].maximum : sample.maximum;
            state.controller_limits.push_back({JointPosition(minimum), JointPosition(maximum),
                settings.maximum_velocity, 0});
        }
        state.sample_id = ++sample_id_;
        state.source_time_s = std::chrono::duration<double>(state.received_at.time_since_epoch()).count();
        state.valid = true;
        if (std::chrono::duration<double>(Clock::now() - state.received_at).count() > definition_.maximum_age_s) {
            throw std::runtime_error("SO101 complete arm feedback exceeded maximum_age_s");
        }
        return state;
    }
    void Enable() override {
        RequireCalibration();
        const auto state = Read();
        CheckMode(state);
        std::vector<int> ticks;
        for (size_t i = 0; i < kArmJoints; ++i) {
            CheckPosition(i, state.joints_rad[i], samples_[i].position);
            ticks.push_back(samples_[i].position);
        }
        if (state.enabled) return;
        // Set current positions before torque, preventing a retained old target from restarting.
        session_->Target(Ids(), ticks, HoldVelocities());
        const auto prepared = Read();
        if (prepared.fault || prepared.moving || !prepared.position_mode) {
            throw std::runtime_error("SO101 state changed while preparing enable");
        }
        for (size_t i = 0; i < kArmJoints; ++i) {
            if (std::abs(samples_[i].position - ticks[i]) > 2) {
                throw std::runtime_error("SO101 arm moved while preparing enable");
            }
        }
        session_->Torque(Ids(), true);
        const auto after = Read();
        if (!after.enabled || after.fault) throw std::runtime_error("SO101 arm enable not confirmed");
    }
    void Disable() override {
        Read();
        session_->Torque(Ids(), false);
        Read();
        for (const auto& sample : samples_) {
            if (sample.enabled) throw std::runtime_error("SO101 arm disable not confirmed");
        }
    }
    void Submit(const JointPoint& point) override {
        RequireCalibration();
        if (point.target_rad.size() != kArmJoints || !std::isfinite(point.speed_scale) ||
            point.speed_scale <= 0 || point.speed_scale > 1 || point.resend_interval_s != 0) {
            throw CommandRejected("SO101 requires five joints, speed_scale in (0,1], no resend interval");
        }
        if (point.speed_scale < GetCapabilities().minimum_speed_scale) {
            throw CommandRejected("SO101 speed is below one encoder tick per second");
        }
        std::vector<int> ticks;
        for (size_t i = 0; i < kArmJoints; ++i) {
            const int target = JointTicks(point.target_rad[i]);
            CheckPosition(i, point.target_rad[i], target);
            ticks.push_back(target);
        }
        const auto state = Read();
        CheckMode(state);
        if (!state.enabled) throw CommandRejected("SO101 arm torque is disabled; explicitly enable first");
        session_->Target(Ids(), ticks, Velocities(point.speed_scale));
    }
    bool Hold() override {
        const auto initial = Read();
        if (!initial.enabled || !initial.position_mode || initial.fault) return false;
        std::vector<int> target;
        for (const auto& sample : samples_) target.push_back(sample.position);
        session_->Target(Ids(), target, HoldVelocities());
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        size_t stationary = 0;
        auto previous = initial.joints_rad;
        while (Clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            const auto state = Read();
            bool still = state.enabled && state.position_mode && !state.fault && !state.moving;
            for (size_t i = 0; i < kArmJoints; ++i) {
                still = still && std::abs(state.joints_rad[i] - previous[i]) <= 0.0031;
            }
            stationary = still ? stationary + 1 : 0;
            if (stationary >= 2) return true;
            previous = state.joints_rad;
        }
        return false;
    }
private:
    void RequireCalibration() const {
        if (!session_->Config().arm_calibrated) {
            throw CommandRejected("SO101 arm_calibration is required before enabling or moving");
        }
    }
    void CheckMode(const ArmState& state) const {
        if (!state.position_mode || state.fault || state.moving) {
            throw CommandRejected("SO101 requires stationary, fault-free position-mode feedback");
        }
    }
    void CheckPosition(size_t index, double radians, int ticks) const {
        const auto& limit = definition_.joints[index];
        const auto& calibration = session_->Config().calibration[index];
        if (!std::isfinite(radians) || radians < limit.minimum_rad || radians > limit.maximum_rad ||
            ticks < calibration.minimum || ticks > calibration.maximum) {
            throw CommandRejected("SO101 target is outside configured/calibrated limits: " + limit.name);
        }
    }
    std::vector<uint8_t> Ids() const {
        const auto& ids = session_->Config().ids;
        return {ids.begin(), ids.end()};
    }
    std::vector<double> Velocities(double scale) const {
        return std::vector<double>(kArmJoints, session_->Config().maximum_velocity * scale);
    }
    std::vector<double> HoldVelocities() const {
        return std::vector<double>(kArmJoints, std::min(session_->Config().maximum_velocity, 0.05));
    }
    DeviceConfig definition_;
    std::shared_ptr<SharedSession> session_;
    std::array<Sample, kArmJoints> samples_{};
    bool connected_ = false;
    uint64_t sample_id_ = 0;
};
}  // namespace

std::unique_ptr<Arm> MakeArm(const DeviceConfig& definition, std::shared_ptr<SharedSession> session) {
    return std::make_unique<So101Arm>(definition, std::move(session));
}
}  // namespace spacemit::manipulation::so101
