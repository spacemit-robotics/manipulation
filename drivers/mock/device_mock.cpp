// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Deterministic offline device assembly, never a hardware connection fallback.
#include "manipulation/device.h"
#include <stdexcept>
namespace spacemit::manipulation {
namespace {
class MockArm final : public Arm {
public:
    explicit MockArm(std::size_t count) : joints_(count, 0) {}
    ArmCapabilities GetCapabilities() const override { return {true, true, true, 0, true, true, true}; }
    void Connect() override { connected_ = true; }
    void Disconnect() noexcept override { connected_ = false; }
    ArmState Read() override {
        if (!connected_) throw std::runtime_error("mock disconnected");
        ArmState state;
        state.joints_rad = joints_;
        state.received_at = Clock::now();
        state.source_time_s = std::chrono::duration<double>(state.received_at.time_since_epoch()).count();
        state.sample_id = ++sample_;
        state.valid = true;
        state.enabled = enabled_;
        state.position_mode = true;
        state.firmware = "offline-mock-v1";
        return state;
    }
    void Enable() override { enabled_ = true; }
    void Disable() override { enabled_ = false; }
    void Submit(const JointPoint& point) override { joints_ = point.target_rad; }
    void SubmitPath(const JointPath& path) override { joints_ = path.waypoints_rad.back(); }
    void SubmitTrajectory(const JointTrajectory& trajectory) override { joints_ = trajectory.positions_rad.back(); }
    bool Hold() override { return connected_ && enabled_; }

private:
    std::vector<double> joints_;
    std::uint64_t sample_ = 0;
    bool connected_ = false;
    bool enabled_ = true;
};

class MockEffector final : public EndEffector {
public:
    MockEffector(std::size_t channels, bool feedback) : positions_(channels, 0), feedback_(feedback) {}
    EffectorCapabilities GetCapabilities() const override { return {feedback_, 120, true, true}; }
    void Connect() override { connected_ = true; }
    void Disconnect() noexcept override { connected_ = false; }
    EffectorState Read() override {
        if (!connected_) throw std::runtime_error("mock effector not connected");
        return {feedback_ ? positions_ : std::vector<double>{}, Clock::now(), ++sample_,
            true, false, false, "mock", enabled_};
    }
    void Enable() override { enabled_ = true; }
    void Disable() override { enabled_ = false; }
    void Submit(const EffectorPoint& point) override { positions_ = point.positions; }
    bool Hold() override { return connected_ && enabled_ && feedback_; }

private:
    std::vector<double> positions_;
    bool feedback_;
    bool connected_ = false;
    bool enabled_ = true;
    std::uint64_t sample_ = 0;
};
}  // namespace

void RegisterMockDevice(DeviceRegistry* registry) {
    registry->Register({"mock_device", 1,
        [](const DeviceDefinition&, const DriverOptions& options) {
            if (!options.empty()) throw std::runtime_error("mock_device has no options");
        },
        [](const DeviceDefinition& definition, const DriverOptions&) {
            Devices devices;
            devices.arm = std::make_unique<MockArm>(definition.arm.joints.size());
            if (!definition.effector.channels.empty()) {
                devices.effector = std::make_unique<MockEffector>(definition.effector.channels.size(),
                    definition.effector.feedback == EffectorFeedback::kPosition);
            }
            return devices;
        }});
}
}  // namespace spacemit::manipulation
