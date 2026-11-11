// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Public embedding facade: persistent shared world, snapshots and explicit disconnected reset.
#include "manipulation/mujoco_runtime.h"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "manipulation/device_plan.h"
#include "mujoco_world.h"

namespace m = spacemit::manipulation;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "usage: mujoco_runtime_test FIXTURE_DIRECTORY");
        const auto plan =
                m::LoadDevicePlan((std::filesystem::path(argv[1]) / "device.yaml").string(),
                        m::MakeBuiltinDeviceRegistry());
        m::MujocoRuntime runtime(plan.definition, plan.composition.arm_options);
        runtime.Start();
        const auto first = runtime.Snapshot();
        auto devices = runtime.CreateDevices();
        devices.arm->Connect();
        devices.effector->Connect();
        bool refused = false;
        try {
            runtime.Reset();
        } catch (const m::CommandRejected&) {
            refused = true;
        }
        Check(refused, "active control roles allowed a world reset");
        devices.arm->Submit({{0.1, -0.1}, 1});
        const auto deadline = m::Clock::now() + std::chrono::seconds(3);
        while (m::Clock::now() < deadline) {
            const auto state = devices.arm->Read();
            if (!state.command_active && !state.moving &&
                    std::abs(state.joints_rad[0] - 0.1) < 0.005)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        Check(devices.arm->Hold(), "embedded arm hold failed");
        const auto moved = runtime.Snapshot();
        Check(moved.Joint("second_joint").position > 0.08, "embedded roles used a different world");
        Check(std::abs(first.Joint("second_joint").position) < 1e-12,
                "snapshot was mutated by later physics");
        devices.arm->Submit({{0.8, -0.8}, 1});
        devices.effector->Submit({{0.1}, 2});
        const auto moving_deadline = m::Clock::now() + std::chrono::seconds(1);
        while (m::Clock::now() < moving_deadline &&
                (!devices.arm->Read().moving || !devices.effector->Read().moving)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        Check(devices.arm->Read().moving && devices.effector->Read().moving,
                "both roles must be moving before the runtime-wide hold");
        const auto tool_before_hold = devices.effector->Read().positions[0];
        auto held = std::async(std::launch::async, [&] { return runtime.Hold(); });
        const auto freeze_deadline = m::Clock::now() + std::chrono::milliseconds(200);
        while (m::Clock::now() < freeze_deadline && devices.effector->Read().command_active) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        Check(!devices.effector->Read().command_active,
                "runtime hold waited for the arm before freezing the effector target");
        Check(held.get(), "runtime-wide hold did not verify both roles stopped");
        Check(!devices.arm->Read().moving && !devices.effector->Read().moving,
                "runtime hold reported success while a role still moved");
        Check(std::abs(devices.effector->Read().positions[0] - tool_before_hold) < 0.04,
                "effector continued chasing the old target while arm hold settled");
        devices.arm->Disconnect();
        devices.effector->Disconnect();
        const auto before = runtime.Snapshot().Time();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        Check(runtime.Snapshot().Time() > before, "control disconnect destroyed runtime world");
        runtime.Reset();
        Check(std::abs(runtime.Snapshot().Joint("second_joint").position) < 1e-12,
                "explicit reset did not restore configured keyframe");
        runtime.Stop();
        Check(moved.IsValid() && moved.Joint("second_joint").position > 0.08,
                "snapshot did not survive runtime stop");
        std::cout << "MuJoCo embedding contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
