// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Real physics contract through public roles, not direct engine or MuJoCo APIs.
#include "manipulation/device_plan.h"
#include "manipulation/mujoco_driver.h"

#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace m = spacemit::manipulation;

namespace {
void Check(bool value, const char* detail) {
    if (!value) throw std::runtime_error(detail);
}

template <typename Operation>
void Reject(Operation operation, const char* detail) {
    bool refused = false;
    try {
        operation();
    } catch (const std::exception&) {
        refused = true;
    }
    Check(refused, detail);
}

void Near(double actual, double expected, double tolerance, const char* detail) {
    Check(std::abs(actual - expected) <= tolerance, detail);
}

void TestShared(
        const m::DevicePlan& plan, const m::DeviceRegistry& registry, const std::string& locks) {
    auto devices = m::CreatePlanDevices(plan, registry);
    Check(devices.arm->GetCapabilities().joint_point, "point capability missing");
    Check(!devices.arm->GetCapabilities().joint_path &&
                    !devices.arm->GetCapabilities().joint_trajectory &&
                    !devices.arm->GetCapabilities().enable &&
                    !devices.arm->GetCapabilities().disable,
            "simulation invented unsupported capabilities");
    // Connecting the second role must not reset an already running world.
    devices.arm->Connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const auto before = devices.arm->Read();
    devices.effector->Connect();
    const auto after = devices.arm->Read();
    Check(after.source_time_s >= before.source_time_s && after.sample_id >= before.sample_id,
            "shared effector connection reset the world");
    devices.arm->Disconnect();
    Check(devices.effector->Read().valid, "first role disconnect destroyed shared world");
    devices.effector->Disconnect();
    devices.arm->Connect();
    Check(devices.arm->Read().sample_id > after.sample_id, "reconnect regressed feedback sequence");
    devices.arm->Disconnect();

    m::DeviceSession session(plan.definition, m::CreatePlanDevices(plan, registry), locks);
    session.Connect();
    const auto initial = session.ReadArm();
    Check(initial.joints_rad.size() == 2 && initial.valid, "mapped joints missing");
    Near(initial.joints_rad[0], 0, 1e-12, "free body shifted named joint mapping");
    Near(session.ReadEffector().positions[0], 0.5, 1e-12, "keyframe/tool normalization failed");
    m::DeviceSession competitor(plan.definition, m::CreatePlanDevices(plan, registry), locks);
    Reject([&] { competitor.Connect(); }, "duplicate resource owner accepted");
    Reject([&] { session.EnableArm(); }, "unsupported enable accepted");
    Reject([&] { session.DisableEffector(); }, "unsupported disable accepted");
    Check(!session.Latched(), "unsupported operation latched session");
    const auto moved = session.MoveArm({{0.18, -0.11}, 0.8});
    Check(moved.status == m::Result::kSucceeded, "physical arm point failed");
    const auto actual = session.ReadArm();
    Near(actual.joints_rad[0], 0.18, 0.005, "reordered second joint target missing");
    Near(actual.joints_rad[1], -0.11, 0.005, "reordered first joint target missing");
    Check(!actual.moving && !actual.command_active, "arrival preceded real stop/dispatch");
    const auto tool = session.MoveEffector({{0.25}, 0.15});
    Check(tool.status == m::Result::kSucceeded, "tendon effector move failed");
    Near(session.ReadEffector().positions[0], 0.25, 0.01, "effector normalization inverted");
    Check(!session.ReadEffector().holding.has_value(), "invented grasp evidence");
    session.Disconnect();
    competitor.Connect();
    competitor.Disconnect();
}

void TestConcurrentRoleLifecycle(const m::DevicePlan& plan, const m::DeviceRegistry& registry) {
    auto devices = m::CreatePlanDevices(plan, registry);
    std::atomic<bool> failed{false};
    const auto exercise = [&](auto* role) {
        try {
            for (unsigned attempt = 0; attempt < 40; ++attempt) {
                role->Connect();
                Check(role->Read().valid, "concurrent role connection invalid");
                role->Disconnect();
            }
        } catch (...) {
            failed = true;
        }
    };
    std::thread arm(exercise, devices.arm.get());
    std::thread effector(exercise, devices.effector.get());
    arm.join();
    effector.join();
    Check(!failed, "shared final disconnect raced another role connection");
}

void TestCancellationAndTimeout(
        m::DevicePlan plan, const m::DeviceRegistry& registry, const std::string& locks) {
    m::DeviceSession session(plan.definition, m::CreatePlanDevices(plan, registry), locks);
    session.Connect();
    const auto began = m::Clock::now();
    const auto cancelled = session.MoveArm({{0.8, -0.8}, 0.2},
            [&] { return m::Clock::now() - began >= std::chrono::milliseconds(120); });
    Check(cancelled.status == m::Result::kCancelled, "cancellation did not verify physical stop");
    Check(!session.ReadArm().moving && !session.Latched(), "cancel did not remain reusable");
    const auto held = session.ReadArm().joints_rad;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    Near(session.ReadArm().joints_rad[0], held[0], 0.002, "cancel kept streaming old target");
    Check(session.FollowArm({{0.1, -0.1}, 0.4}).status == m::Result::kSucceeded,
            "follow did not accept a target");
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    Check(session.FollowArm({{-0.1, 0.1}, 0.4}).status == m::Result::kSucceeded,
            "follow replacement rejected own motion");
    Check(session.HoldArm().status == m::Result::kSucceeded, "follow hold unconfirmed");
    session.Disconnect();

    plan.definition.arm.timeout_s = 0.05;
    m::DeviceSession timeout(plan.definition, m::CreatePlanDevices(plan, registry), locks);
    timeout.Connect();
    const auto result = timeout.MoveArm({{0.9, -0.9}, 0.1});
    Check(result.status == m::Result::kTimedOut && timeout.Latched(),
            "deadline did not hold and latch simulation");
    Reject([&] { timeout.MoveArm({{0, 0}, 1}); }, "latched simulation accepted a command");
    timeout.Disconnect();
    timeout.Connect();
    Check(!timeout.Latched(), "explicit reconnect did not clear latch");
    timeout.Disconnect();
}

void TestCachedSamples(const m::DevicePlan& plan, const m::DeviceRegistry& registry) {
    auto devices = m::CreatePlanDevices(plan, registry);
    devices.arm->Connect();
    auto previous = devices.arm->Read();
    bool saw_cached = false;
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        const auto sample = devices.arm->Read();
        if (sample.sample_id == previous.sample_id) {
            Check(sample.received_at == previous.received_at,
                    "cached feedback was stamped as a new sample");
            saw_cached = true;
            break;
        }
        previous = sample;
    }
    Check(saw_cached, "every read fabricated new feedback");
    const auto invalid_targets = {std::vector<double>{0}, std::vector<double>{2, 0},
            std::vector<double>{std::numeric_limits<double>::quiet_NaN(), 0}};
    for (const auto& target : invalid_targets) {
        Reject([&] { devices.arm->Submit({target, 1}); }, "invalid driver target accepted");
    }
    Reject([&] { devices.arm->Submit({{0, 0}, 0}); }, "zero driver speed accepted");
    Reject([&] { devices.arm->SubmitPath({{{0, 0}}}); }, "unsupported path accepted");
    Reject([&] { devices.arm->SubmitTrajectory({}); }, "unsupported trajectory accepted");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    Near(devices.arm->Read().joints_rad[0], 0, 1e-12, "invalid targets changed physics control");
    devices.arm->Disconnect();
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "usage: manipulation_mujoco_test FIXTURE_DIRECTORY");
        const auto registry = m::MakeBuiltinDeviceRegistry();
        const auto plan = m::LoadDevicePlan(
                (std::filesystem::path(argv[1]) / "device.yaml").string(), registry);
        char pattern[] = "/tmp/manipulation-mujoco-contract-XXXXXX";
        const auto directory = mkdtemp(pattern);
        Check(directory != nullptr, "cannot create private lock directory");
        const std::filesystem::path temporary(directory);
        try {
            TestCachedSamples(plan, registry);
            TestConcurrentRoleLifecycle(plan, registry);
            Check(m::IsMujocoArmDriver("mujoco_ur5e") &&
                            m::IsMujocoArmDriver("mujoco_mapped_arm") &&
                            !m::IsMujocoArmDriver("mujoco") &&
                            !m::IsMujocoArmDriver("mujoco_robotiq_2f85") &&
                            m::IsMujocoEffectorDriver("mujoco_robotiq_2f85") &&
                            m::IsMujocoEffectorDriver("mujoco_mapped_effector") &&
                            !m::IsMujocoEffectorDriver("mujoco") &&
                            !m::IsMujocoEffectorDriver("mujoco_ur5e"),
                    "MuJoCo role classification accepted an unknown or mismatched driver");
            TestShared(plan, registry, directory);
            TestCancellationAndTimeout(plan, registry, directory);
        } catch (...) {
            std::filesystem::remove_all(temporary);
            throw;
        }
        std::filesystem::remove_all(temporary);
        std::cout << "MuJoCo public driver contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
