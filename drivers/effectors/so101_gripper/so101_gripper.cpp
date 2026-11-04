// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// One calibrated gripper channel: 0 closed, 1 open, in either encoder direction. A closing move that
// stops short of its target under load (grip_load_threshold) reports holding, as the former grasp
// component's HOLDING state did.
#include "so101_session.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace spacemit::manipulation::so101 {
namespace {
class So101Gripper final : public EndEffector {
public:
    So101Gripper(EffectorConfig definition, std::shared_ptr<SharedSession> session)
        : definition_(std::move(definition)), session_(std::move(session)) {}
    ~So101Gripper() override { Disconnect(); }
    EffectorCapabilities GetCapabilities() const override { return {true, 120, true, true, true}; }
    void Connect() override {
        if (connected_) throw std::runtime_error("SO101 gripper already connected");
        if (!session_->Config().gripper_calibrated) throw std::runtime_error("SO101 gripper is uncalibrated");
        session_->Connect();
        connected_ = true;
        connected_offset_.reset();
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
    EffectorState Read() override {
        if (!connected_) throw std::runtime_error("SO101 gripper disconnected");
        const auto started = Clock::now();
        const auto& settings = session_->Config();
        sample_ = session_->Read(settings.gripper_id);
        if ((settings.gripper_offset && *settings.gripper_offset != sample_.offset) ||
            (connected_offset_ && *connected_offset_ != sample_.offset)) {
            throw std::runtime_error("SO101 gripper homing offset differs from calibration or changed during use");
        }
        connected_offset_ = sample_.offset;
        if (std::chrono::duration<double>(Clock::now() - started).count() > definition_.maximum_age_s) {
            throw std::runtime_error("SO101 gripper feedback exceeded maximum_age_s");
        }
        if (std::min(settings.closed_ticks, settings.open_ticks) < sample_.minimum ||
            std::max(settings.closed_ticks, settings.open_ticks) > sample_.maximum) {
            throw std::runtime_error("SO101 gripper calibration exceeds installed hardware limits");
        }
        double position = (sample_.position - settings.closed_ticks) /
            (settings.open_ticks - settings.closed_ticks);
        // Preserve out-of-range evidence; a narrow overshoot is bounded to encoder quantization only.
        if (position < -0.02 || position > 1.02) {
            throw std::runtime_error("SO101 gripper feedback is outside its calibrated range");
        }
        position = std::clamp(position, 0.0, 1.0);
        // Holding long enough trips overload protection, which only limits output to Overload_Torque:
        // expected while gripping an object. Every other servo error is a fault.
        const bool fault = (sample_.error & ~kOverloadError) != 0 || !sample_.position_mode;
        holding_ = target_.has_value() && sample_.enabled && !sample_.moving &&
            *target_ < position - definition_.tolerance &&  // Stopped short of a closing target.
            std::abs(sample_.load) >= settings.grip_load_threshold;
        EffectorState state{{position}, started, ++sample_id_, true, sample_.moving, fault, "", sample_.enabled};
        state.loads = {std::clamp(sample_.load / 1000.0, -1.0, 1.0)};
        state.holding = holding_;
        return state;
    }
    void Enable() override {
        const auto state = Read();
        if (state.fault || state.moving) throw CommandRejected("SO101 gripper must be stationary and fault-free");
        if (sample_.torque_limit > kGripperTorqueLimit || sample_.protection_current > kGripperProtectionCurrent) {
            throw CommandRejected("SO101 gripper protection is not configured (Max_Torque_Limit <= " +
                std::to_string(kGripperTorqueLimit) + ", Protection_Current <= " +
                std::to_string(kGripperProtectionCurrent) + " mA); release torque and run so101-calibrate configure");
        }
        if (sample_.enabled) return;
        const int position = sample_.position;
        session_->Target({Id()}, {position}, {std::min(session_->Config().maximum_velocity, 0.05)});
        const auto prepared = Read();
        if (prepared.fault || prepared.moving || std::abs(sample_.position - position) > 2) {
            throw std::runtime_error("SO101 gripper moved while preparing enable");
        }
        target_ = prepared.positions[0];
        session_->Torque({Id()}, true);
        const auto after = Read();
        if (after.enabled != true || after.fault) throw std::runtime_error("SO101 gripper enable not confirmed");
    }
    void Disable() override {
        Read();
        target_.reset();
        session_->Torque({Id()}, false);
        if (Read().enabled != false) throw std::runtime_error("SO101 gripper disable not confirmed");
    }
    void Submit(const EffectorPoint& point) override {
        if (point.positions.size() != 1 || !std::isfinite(point.positions[0]) ||
            point.positions[0] < 0 || point.positions[0] > 1 || !std::isfinite(point.duration_s) ||
            point.duration_s <= 0 || point.duration_s > 120) {
            throw CommandRejected("SO101 gripper needs one position in [0,1] and duration in (0,120]");
        }
        const auto state = Read();
        if (state.fault || !sample_.enabled) {
            throw CommandRejected("SO101 gripper requires fault-free feedback and explicit enable");
        }
        const auto& settings = session_->Config();
        const int target = static_cast<int>(std::lround(settings.closed_ticks +
            point.positions[0] * (settings.open_ticks - settings.closed_ticks)));
        const double distance = std::abs(JointPosition(target) - JointPosition(sample_.position));
        const double speed = std::min(settings.maximum_velocity, std::max(0.01, distance / point.duration_s));
        target_ = point.positions[0];
        session_->Target({Id()}, {target}, {speed});
    }
    bool Hold() override {
        const auto initial = Read();
        if (!sample_.enabled || initial.fault) return false;
        // A held object keeps its closing goal: retargeting to the present position would release it.
        if (!holding_) {
            session_->Target({Id()}, {sample_.position}, {std::min(session_->Config().maximum_velocity, 0.05)});
            target_ = initial.positions[0];
        }
        auto previous = initial.positions[0];
        size_t stationary = 0;
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        while (Clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            const auto state = Read();
            const bool still = sample_.enabled && !state.fault && !state.moving &&
                std::abs(state.positions[0] - previous) < 0.01;
            stationary = still ? stationary + 1 : 0;
            if (stationary >= 2) return true;
            previous = state.positions[0];
        }
        return false;
    }
private:
    uint8_t Id() const { return session_->Config().gripper_id; }
    EffectorConfig definition_;
    std::shared_ptr<SharedSession> session_;
    Sample sample_;
    std::optional<int> connected_offset_;
    std::optional<double> target_;  // Last commanded normalized position.
    bool holding_ = false;
    bool connected_ = false;
    uint64_t sample_id_ = 0;
};
}  // namespace

std::unique_ptr<EndEffector> MakeGripper(const EffectorConfig& definition,
    std::shared_ptr<SharedSession> session) {
    return std::make_unique<So101Gripper>(definition, std::move(session));
}
}  // namespace spacemit::manipulation::so101
