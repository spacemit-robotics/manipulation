// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Offline extension example. Replace this class, never a central model switch.
#include "manipulation/device.h"
#include <stdexcept>
namespace spacemit::manipulation {
namespace {
class BenchArm final : public Arm {
public:
    ArmCapabilities GetCapabilities() const override { return {true, false, true}; }
    void Connect() override { connected_ = true; }
    void Disconnect() noexcept override { connected_ = false; }
    ArmState Read() override {
        if (!connected_) throw std::runtime_error("bench disconnected");
        ArmState state;
        state.joints_rad = {position_};
        state.received_at = Clock::now();
        state.sample_id = ++sequence_;
        state.valid = state.enabled = state.position_mode = true;
        state.firmware = "offline-bench-example";
        return state;
    }
    void Submit(const JointPoint& point) override { position_ = point.target_rad.at(0); }
    bool Hold() override { return connected_; }
private:
    bool connected_ = false;
    double position_ = 0;
    std::uint64_t sequence_ = 0;
};
}  // namespace
void RegisterBenchDevice(DeviceRegistry* registry) {
    registry->Register({"example_bench_mock", 1,
        [](const DeviceDefinition& definition, const DriverOptions& options) {
            if (definition.arm.joints.size() != 1 || !definition.effector.channels.empty() || !options.empty()) {
                throw std::runtime_error("bench example requires one joint, no tool/options");
            }
        },
        [](const DeviceDefinition&, const DriverOptions&) {
            Devices devices;
            devices.arm = std::make_unique<BenchArm>();
            return devices;
        }});
}
}  // namespace spacemit::manipulation
