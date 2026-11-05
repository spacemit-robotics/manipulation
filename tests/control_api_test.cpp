// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Capability, torque permission, hold and relative-joint contracts without hardware.
#include "manipulation/device.h"

#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace m = spacemit::manipulation;

namespace {
void Check(bool condition, const char* detail) {
    if (!condition) throw std::runtime_error(detail);
}

void Refused(const std::function<void()>& operation) {
    bool rejected = false;
    try {
        operation();
    } catch (const m::CommandRejected&) {
        rejected = true;
    }
    Check(rejected, "operation was not rejected before a write");
}

void Invalid(const std::function<void()>& operation) {
    bool rejected = false;
    try {
        operation();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, "invalid operation accepted");
}

class CountingArm final : public m::Arm {
public:
    m::ArmCapabilities GetCapabilities() const override { return capabilities; }

    void Connect() override { ++connections; }

    void Disconnect() noexcept override {}

    m::ArmState Read() override {
        if (reject_reads) throw m::CommandRejected("snapshot refused");
        m::ArmState state;
        state.joints_rad = {position};
        state.received_at = m::Clock::now();
        state.sample_id = ++sequence_;
        state.valid = true;
        state.enabled = enabled;
        state.position_mode = true;
        state.fault = fault;
        state.moving = moving;
        return state;
    }

    void Enable() override {
        if (reject) throw m::CommandRejected("enable refused");
        ++writes;
        enabled = true;
        reject_reads = reject_after_write;
    }

    void Disable() override {
        if (reject) throw m::CommandRejected("disable refused");
        ++writes;
        enabled = false;
    }

    void Submit(const m::JointPoint& point) override {
        ++writes;
        position = point.target_rad.at(0);
    }

    bool Hold() override {
        if (reject) throw m::CommandRejected("hold refused");
        ++holds;
        return confirm_hold;
    }

    m::ArmCapabilities capabilities{true, false, false};
    int connections = 0, writes = 0, holds = 0;
    bool enabled = true, fault = false, moving = false, reject = false, confirm_hold = true;
    bool reject_after_write = false, reject_reads = false;
    double position = 0;

private:
    std::uint64_t sequence_ = 0;
};

class CountingEffector final : public m::EndEffector {
public:
    m::EffectorCapabilities GetCapabilities() const override { return capabilities; }

    void Connect() override { ++connections; }

    void Disconnect() noexcept override {}

    m::EffectorState Read() override {
        m::EffectorState state;
        state.positions = {0};
        state.received_at = m::Clock::now();
        state.sample_id = ++sequence_;
        state.valid = true;
        state.enabled = enabled;
        state.fault = fault;
        return state;
    }

    void Enable() override {
        if (reject) throw m::CommandRejected("effector enable refused");
        ++writes;
        enabled = true;
    }

    void Disable() override {
        if (reject) throw m::CommandRejected("effector disable refused");
        ++writes;
        enabled = false;
    }

    void Submit(const m::EffectorPoint&) override { ++writes; }

    bool Hold() override {
        if (reject) throw m::CommandRejected("effector hold refused");
        ++holds;
        return confirm_hold;
    }

    m::EffectorCapabilities capabilities;
    std::optional<bool> enabled;
    int connections = 0, writes = 0, holds = 0;
    bool fault = false, reject = false, confirm_hold = true;

private:
    std::uint64_t sequence_ = 0;
};

void TestCapabilitiesAndRejections(const std::string& directory) {
    m::DeviceDefinition definition;
    definition.arm = {"arm", "control_api", {{"joint", -1, 1}}};
    definition.effector = {"tool", {"finger"}};
    auto arm = std::make_unique<CountingArm>();
    auto tool = std::make_unique<CountingEffector>();
    auto* arm_raw = arm.get();
    auto* tool_raw = tool.get();
    m::DeviceSession session(definition, {std::move(arm), std::move(tool)}, directory);
    Check(!session.GetArmCapabilities().disable && !session.GetEffectorCapabilities().enable,
          "old driver acquired new capabilities implicitly");
    Check(arm_raw->connections == 0 && tool_raw->connections == 0,
          "capabilities connected hardware");
    session.Connect();
    Check(!session.ReadEffector().enabled.has_value(), "unknown torque state reported as disabled");
    Refused([&] { session.EnableArm(); });
    Refused([&] { session.DisableArm(); });
    Refused([&] { session.EnableEffector(); });
    Refused([&] { session.DisableEffector(); });
    Refused([&] { session.HoldArm(); });
    Refused([&] { session.HoldEffector(); });
    Check(arm_raw->writes == 0 && arm_raw->holds == 0 && tool_raw->writes == 0 &&
                  tool_raw->holds == 0,
          "unsupported command reached a driver write");
    Check(!session.Latched(), "unsupported operation latched the device");

    for (double step :
         {2.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        Invalid([&] { session.JogArm(0, step); });
    }
    Invalid([&] { session.JogArm(1, 0.1); });
    Invalid([&] { session.JogArm(0, 0.1, 0); });
    Check(arm_raw->writes == 0 && !session.Latched(), "invalid jog wrote or latched");
    Check(session.JogArm(0, 0.1).status == m::Result::kSucceeded && arm_raw->position == 0.1,
          "relative joint step failed");
    const auto writes = arm_raw->writes;
    Invalid([&] { session.JogArm(0, 0.95); });
    Check(arm_raw->writes == writes, "relative joint step bypassed configured limits");

    arm_raw->capabilities.enable = true;
    arm_raw->capabilities.disable = true;
    arm_raw->capabilities.verified_hold = true;
    tool_raw->capabilities = {true, 120, true, true};
    arm_raw->reject = tool_raw->reject = true;
    Refused([&] { session.EnableArm(); });
    Refused([&] { session.DisableArm(); });
    Refused([&] { session.EnableEffector(); });
    Refused([&] { session.DisableEffector(); });
    Check(!session.Latched() && arm_raw->writes == writes && tool_raw->writes == 0,
          "driver refusal mutated command state");
    // A refused hold establishes no stop: it reports stop_unknown and latches until reconnect.
    Check(session.HoldArm().status == m::Result::kStopUnknown && session.Latched(),
          "refused arm hold did not latch");
    session.Disconnect();
    session.Connect();
    Check(session.HoldEffector().status == m::Result::kStopUnknown && session.Latched(),
          "refused tool hold did not latch");
    session.Disconnect();
    session.Connect();
    arm_raw->reject = tool_raw->reject = false;
    session.DisableEffector();
    session.DisableArm();
    Refused([&] { session.HoldArm(); });
    Refused([&] { session.HoldEffector(); });
    Check(!session.ReadArm().enabled && !*session.ReadEffector().enabled,
          "hold re-enabled a disabled device");
    // A refusal after the driver wrote (here the confirmation read) is not "nothing written".
    arm_raw->reject_after_write = true;
    Invalid([&] { session.EnableArm(); });
    Check(session.Latched(), "unconfirmed enable did not latch");
    arm_raw->reject_after_write = arm_raw->reject_reads = false;
    session.Disconnect();
    session.Connect();
    Check(session.ReadArm().enabled, "enable was not written");
    session.EnableEffector();
    Check(session.HoldArm().status == m::Result::kSucceeded &&
                  session.HoldEffector().status == m::Result::kSucceeded,
          "enabled hold failed");
    Check(session.ReadArm().enabled && *session.ReadEffector().enabled, "hold released torque");

    arm_raw->fault = true;
    Check(session.HoldArm().status == m::Result::kSucceeded && session.Latched(),
          "hold did not preserve an observed arm fault");
    Check(session.HoldEffector().status == m::Result::kSucceeded && session.Latched(),
          "tool hold cleared the assembly fault");
    arm_raw->fault = false;
    Invalid([&] { session.JogArm(0, 0.1); });
    session.Disconnect();
    session.Connect();
    tool_raw->confirm_hold = false;
    Check(session.HoldEffector().status == m::Result::kStopUnknown && session.Latched(),
          "unconfirmed tool hold did not latch");
    tool_raw->confirm_hold = true;
    Check(session.HoldEffector().status == m::Result::kSucceeded && session.Latched(),
          "later confirmed hold cleared an unknown-stop latch");
}

void TestMockTorque(const std::string& directory) {
    auto registry = m::MakeBuiltinDeviceRegistry();
    m::DeviceDefinition definition;
    definition.arm = {"arm", "mock_torque", {{"joint", -1, 1}}};
    definition.effector = {"hand", {"finger"}};
    m::DeviceSession session(definition, registry.Create("mock_device", definition, {}), directory);
    session.Connect();
    session.DisableEffector();
    Invalid([&] { session.MoveEffector({{0.5}, 0.1}); });
    Check(!session.Latched() && session.ReadEffector().positions == std::vector<double>{1},
          "disabled effector moved or latched");
    session.EnableEffector();
    session.DisableArm();
    Invalid([&] { session.JogArm(0, 0.1); });
    Check(!session.Latched() && session.ReadArm().joints_rad == std::vector<double>{0},
          "disabled arm moved or latched");
    session.EnableArm();
    Check(session.JogArm(0, -0.1).status == m::Result::kSucceeded, "re-enabled mock cannot move");
    Check(session.MoveEffector({{0.5}, 0.1}).status == m::Result::kSucceeded,
          "re-enabled mock tool cannot move");
    // A measured move longer than the effector timeout_s could only time out: refused unwritten.
    Refused([&] { session.MoveEffector({{1}, definition.effector.timeout_s + 1}); });
    Check(!session.Latched() && session.ReadEffector().positions == std::vector<double>{0.5},
          "over-long effector move was written or latched");
}
}  // namespace

int main() {
    auto pattern =
            (std::filesystem::temp_directory_path() / "manipulation-control-api-XXXXXX").string();
    if (!mkdtemp(pattern.data())) return 1;
    try {
        TestCapabilitiesAndRejections(pattern);
        TestMockTorque(pattern);
        std::filesystem::remove_all(pattern);  // This test's mkdtemp directory only.
        std::cout << "control API contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
