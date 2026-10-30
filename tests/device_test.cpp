// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Exception-safe cancellation for embedded C++ callers, without hardware.
#include "manipulation/device_plan.h"
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <iostream>
#include <stdexcept>
namespace {
class FaultHand final : public spacemit::manipulation::EndEffector {
public:
    bool fault = true;
    void Connect() override {}
    void Disconnect() noexcept override {}
    spacemit::manipulation::EffectorState Read() override {
        spacemit::manipulation::EffectorState state;
        state.positions = {0};
        state.received_at = spacemit::manipulation::Clock::now();
        state.sample_id = ++sequence_;
        state.valid = true;
        state.fault = fault;
        return state;
    }
    void Submit(const spacemit::manipulation::EffectorPoint&) override {
        if (refuse) throw spacemit::manipulation::CommandRejected("injected refusal before write");
    }
    bool refuse = false;
private:
    std::uint64_t sequence_ = 0;
};
}  // namespace
int main() {
    namespace m = spacemit::manipulation;
    auto pattern = (std::filesystem::temp_directory_path() / "manipulation-device-XXXXXX").string();
    const char* directory = mkdtemp(pattern.data());
    if (!directory) return 1;
    try {
        auto registry = m::MakeBuiltinDeviceRegistry();
        m::DeviceDefinition definition;
        definition.arm = {"arm", "mock_device_test", {{"j1", -1, 1}}};
        definition.effector = {"hand", {"finger"}};
        for (bool effector : {false, true}) {
            m::DeviceSession session(definition, registry.Create("mock_device", definition, {}), directory);
            session.Connect();
            unsigned calls = 0;
            auto throwing = [&] {
                if (++calls > 1) throw 7;  // Non-std exception after submission, not only in validation.
                return false;
            };
            auto result = effector ? session.MoveEffector({{0.2}, 0.5}, throwing) :
                session.MoveArm({{0.1}, 0.05}, throwing);
            if (result.status != m::Result::kFault || !session.Latched()) {
                throw std::runtime_error("callback exception did not latch safely");
            }
        }
        {
            auto devices = registry.Create("mock_device", definition, {});
            auto hand = std::make_unique<FaultHand>();
            auto* injected = hand.get();
            devices.effector = std::move(hand);
            m::DeviceSession session(definition, std::move(devices), directory);
            session.Connect();
            session.Diagnose();  // Diagnosis remains possible; it never clears the observed fault latch.
            injected->fault = false;
            if (session.ReadEffector().fault || !session.Latched()) throw std::runtime_error("lost fault latch");
            bool rejected = false;
            try { session.MoveArm({{0.1}, 0.05}); } catch (const std::runtime_error&) { rejected = true; }
            if (!rejected) throw std::runtime_error("cleared feedback silently resumed motion");
            session.Disconnect(); session.Connect();
            if (session.MoveArm({{0.1}, 0.05}).status != m::Result::kSucceeded) {
                throw std::runtime_error("explicit reconnect did not allow inspected mock operation");
            }
            injected->refuse = true;
            bool refused = false;
            try { session.MoveEffector({{0.0}, 0.5}); } catch (const m::CommandRejected&) { refused = true; }
            if (!refused || session.Latched()) throw std::runtime_error("refused effector command latched");
            if (session.MoveArm({{0.0}, 0.05}).status != m::Result::kSucceeded) {
                throw std::runtime_error("refusal before write blocked later motion");
            }
        }
        {
            // Open-loop effector: delivered moves are unverified, never succeeded; a stop is never confirmed.
            auto open = definition;
            open.effector.feedback = m::EffectorFeedback::kNone;
            open.effector.timeout_s = 1;
            m::DeviceSession session(open, registry.Create("mock_device", open, {}), directory);
            session.Connect();
            if (!session.ReadEffector().positions.empty()) throw std::runtime_error("open loop reported positions");
            const auto started = m::Clock::now();
            const auto moved = session.MoveEffector({{0.4}, 0.2});
            if (moved.status != m::Result::kUnverified || session.Latched() ||
                m::Clock::now() - started < std::chrono::milliseconds(200)) {
                throw std::runtime_error("open-loop move not reported as unverified after its duration");
            }
            bool rejected = false;
            try { session.MoveEffector({{0.4}, 2.0}); } catch (const std::runtime_error&) { rejected = true; }
            if (!rejected) throw std::runtime_error("duration beyond timeout_s accepted");
            // Arm-and-hand gesture: effector targets go out on schedule while the trajectory runs.
            const auto here = session.ReadArm().joints_rad;
            const m::JointTrajectory out{{0, 0.05, 0.1}, {here, {here[0] + 0.005}, {here[0] + 0.01}}, 100, 0.0174533,
                1, 0, false};
            const auto gesture_started = m::Clock::now();
            const auto gesture = session.MoveArmTrajectory(out, {}, {{0, {{0.6}, 0.1}}, {0.15, {{0.0}, 0.1}}});
            if (gesture.status != m::Result::kUnverified || session.Latched() ||
                m::Clock::now() - gesture_started < std::chrono::milliseconds(250)) {
                throw std::runtime_error("gesture did not complete after its timed effector targets");
            }
            const auto back = session.ReadArm().joints_rad;
            const m::JointTrajectory still{{0, 0.05}, {back, back}, 100, 0.0174533, 1, 0, false};
            for (const auto& refused : {std::vector<m::TimedEffectorPoint>{{0.2, {{0.1}, 0.1}}, {0.1, {{0.1}, 0.1}}},
                    std::vector<m::TimedEffectorPoint>{{0, {{1.5}, 0.1}}}}) {  // Out of order; outside [0,1].
                bool refused_schedule = false;
                try { session.MoveArmTrajectory(still, {}, refused); }
                catch (const std::runtime_error&) { refused_schedule = true; }
                if (!refused_schedule || session.Latched()) throw std::runtime_error("invalid effector schedule accepted");
            }
            if (session.MoveArm({{0.1}, 0.05}).status != m::Result::kSucceeded) {
                throw std::runtime_error("arm motion blocked after an unverified effector move");
            }
            const auto sequence = m::RunDeviceSteps(session, {
                {"effector_position", {}, {{0.1}, 0.1}}, {"joint_point", {{0.0}, 0.05}, {}}});
            if (sequence.status != m::Result::kUnverified) throw std::runtime_error("sequence hid an unverified step");
            unsigned polls = 0;
            const auto cancel = session.MoveEffector({{0.2}, 0.5}, [&] { return ++polls > 3; });
            if (cancel.status != m::Result::kStopUnknown || !session.Latched()) {
                throw std::runtime_error("open-loop cancel claimed a confirmed stop");
            }
        }
        {
            // Joint paths: validated before any write, completed at the last waypoint.
            m::DeviceDefinition arm_only;
            arm_only.arm = {"arm", "mock_path_test", {{"j1", -1, 1}, {"j2", -1, 1}}};
            m::DeviceSession session(arm_only, registry.Create("mock_device", arm_only, {}), directory);
            session.Connect();
            for (const m::JointPath& refused : {
                    m::JointPath{{}, 0.2, 0.2, 0.05},                    // no waypoint
                    m::JointPath{{{0.1, 0.1}, {1.5, 0.0}}, 0.2, 0.2, 0.05},  // outside the joint limits
                    m::JointPath{{{0.1, 0.1}}, 0.5, 0.2, 0.05},           // above the streaming ceiling
                    m::JointPath{{{0.1}}, 0.2, 0.2, 0.05}}) {             // wrong joint count
                bool rejected = false;
                try { session.MoveArmPath(refused); } catch (const std::runtime_error&) { rejected = true; }
                if (!rejected || session.Latched()) throw std::runtime_error("invalid joint path accepted or latched");
            }
            if (session.ReadArm().joints_rad != std::vector<double>{0, 0}) throw std::runtime_error("refused path moved");
            const auto moved = session.MoveArmPath({{{0.1, 0.1}, {0.2, -0.3}}, 0.2, 0.2, 0.05});
            if (moved.status != m::Result::kSucceeded || session.ReadArm().joints_rad != std::vector<double>{0.2, -0.3}) {
                throw std::runtime_error("joint path did not complete at its last waypoint");
            }
            // Timed trajectories: start at the arm, bounded steps, end at the last knot.
            const std::vector<double> times{0, 0.01, 0.02};
            const auto here = session.ReadArm().joints_rad;
            for (const m::JointTrajectory& refused : {
                    m::JointTrajectory{times, {{0.5, 0.5}, {0.5, 0.5}, {0.5, 0.5}}, 100, 0.0174533, 1, 0},  // not here
                    m::JointTrajectory{times, {here, {here[0] + 0.1, here[1]}, here}, 100, 0.0174533, 1, 0},  // step
                    m::JointTrajectory{{0, 0.02, 0.01}, {here, here, here}, 100, 0.0174533, 1, 0},  // time order
                    m::JointTrajectory{times, {here, here, here}, 100, 0.1, 1, 0}}) {  // step bound above 3 deg
                bool rejected = false;
                try { session.MoveArmTrajectory(refused); } catch (const std::runtime_error&) { rejected = true; }
                if (!rejected || session.Latched()) throw std::runtime_error("invalid trajectory accepted or latched");
            }
            const auto streamed = session.MoveArmTrajectory(
                {times, {here, {here[0] + 0.005, here[1]}, {here[0] + 0.01, here[1]}}, 100, 0.0174533, 1, 0});
            if (streamed.status != m::Result::kSucceeded || session.ReadArm().joints_rad[0] != here[0] + 0.01) {
                throw std::runtime_error("trajectory did not complete at its last knot");
            }
            const auto now = session.ReadArm().joints_rad;
            const auto delivered = session.MoveArmTrajectory(
                {times, {now, {now[0] - 0.005, now[1]}, {now[0] - 0.01, now[1]}}, 100, 0.0174533, 1, 0, false});
            if (delivered.status != m::Result::kSucceeded || delivered.detail.rfind("delivered", 0) != 0) {
                throw std::runtime_error("trajectory without arrival wait did not complete on delivery");
            }
            // Points without arrival wait complete within tolerance; re-send intervals are bounded.
            const auto within = session.MoveArm({{0.05, 0.05}, 0.05, 0, false, 0.05});
            if (within.status != m::Result::kSucceeded || within.detail.rfind("within_tolerance", 0) != 0) {
                throw std::runtime_error("point without arrival wait did not complete within tolerance");
            }
            for (const double interval : {0.01, 0.3, -0.05}) {
                bool rejected = false;
                try { session.MoveArm({{0.0, 0.0}, 0.05, 0, true, interval}); }
                catch (const std::runtime_error&) { rejected = true; }
                if (!rejected || session.Latched()) throw std::runtime_error("invalid re-send interval accepted");
            }
            const auto duration = m::JointPathDuration({0, 0}, {{{0.1, 0.1}, {0.2, -0.3}}, 0.2, 0.2, 0.05});
            if (std::abs(duration - std::max(1.875 * 0.5 / 0.2, std::sqrt(5.774 * 0.5 / 0.2))) > 1e-9) {
                throw std::runtime_error("joint path duration differs from the quintic profile");
            }
        }
        std::filesystem::remove_all(directory);  // Owned mkdtemp test directory, never a user path.
        std::cout << "embedded device exceptions passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
