// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Registered capability composition, offline validation and unchanged legacy task behavior.
#include "manipulation/capability.h"

#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "manipulation/device_plan.h"

namespace {
namespace m = spacemit::manipulation;

void Check(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

template <typename Action>
void Reject(Action action, const char* reason) {
    bool rejected = false;
    try {
        action();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, reason);
}

class OfflineArm final : public m::Arm {
public:
    explicit OfflineArm(unsigned* connected) : connected_(connected) {}

    m::ArmCapabilities GetCapabilities() const override { return {true, false, false, 0.1}; }

    void Connect() override { ++*connected_; }

    void Disconnect() noexcept override {}

    m::ArmState Read() override { throw std::runtime_error("offline read forbidden"); }

    void Submit(const m::JointPoint&) override { throw std::runtime_error("write forbidden"); }

    bool Hold() override { throw std::runtime_error("hold forbidden"); }

private:
    unsigned* connected_;
};

void OfflineChecks(const std::filesystem::path& directory) {
    auto drivers = m::MakeBuiltinDeviceRegistry();
    auto capabilities = m::MakeBuiltinCapabilityRegistry();
    unsigned connected = 0;
    drivers.Register({"offline", 1, [](const m::DeviceDefinition&, const m::DriverOptions&) {},
            [&](const m::DeviceDefinition&, const m::DriverOptions&) {
                return m::Devices{std::make_unique<OfflineArm>(&connected), nullptr};
            }});
    const auto filename = directory / "offline.yaml";
    const std::string header =
            "schema_version: 3\ndevice:\n  driver: offline\n  arm:\n    device_id: arm\n"
            "    resource_id: never_open\n    joints:\n"
            "      - {name: j1, minimum_rad: -1, maximum_rad: 1}\nsteps:\n";
    auto save = [&](const std::string& step) { std::ofstream(filename) << header << step; };
    save("  - {capability: joint_point, target_rad: [0.1], speed_scale: 0.1}\n");
    const auto plan = m::LoadDevicePlan(filename.string(), drivers);
    Check(plan.steps.size() == 1 && connected == 0, "offline parsing connected a driver");
    const auto context = m::DescribePlanCapabilities(plan, drivers);
    bool found_path = false;
    for (const auto& item : capabilities.Describe(context)) {
        if (item.id == "joint_path") {
            found_path = true;
            Check(!item.supported && !item.reason.empty(), "path capability fabricated");
        }
        if (item.id == "effector_position") Check(!item.supported, "absent effector supported");
    }
    Check(found_path && connected == 0, "capability discovery connected a driver");
    for (const auto* step :
            {"  - {capability: joint_point, target_rad: [0.1], speed_scale: 0.05}\n",
                    "  - {capability: joint_path, waypoints_rad: [[0.1]]}\n",
                    "  - {capability: joint_trajectory, times_s: [0, 1], positions_rad: [[0], "
                    "[0.1]]}\n",
                    "  - {capability: effector_position, positions: [0.1], duration_s: 0.1}\n"}) {
        save(step);
        Reject([&] { m::LoadDevicePlan(filename.string(), drivers); },
                "unsupported offline capability accepted");
    }
    Check(connected == 0, "rejected offline task connected a driver");
}

void RegistryChecks(const m::CapabilityContext& context) {
    auto capabilities = m::MakeBuiltinCapabilityRegistry();
    Reject([&] { capabilities.Register(capabilities.Get("joint_point")); },
            "duplicate registration accepted");
    Reject([&] { capabilities.Register({}); }, "incomplete registration accepted");
    for (const auto* yaml :
            {"{capability: joint_point, target_rad: [0.1], speed_scale: 0.05, typo: 0}",
                    "{capability: joint_point, target_rad: [0.1], speed_scale: 0.05, speed_scale: "
                    "0.1}",
                    "{capability: joint_point, target_rad: [.nan], speed_scale: 0.05}",
                    "{capability: joint_point, target_rad: [0.1], speed_scale: 0.05, "
                    "tolerance_rad: 1}",
                    "{capability: joint_jog, joint_index: -1, delta_rad: 0.1}",
                    "{capability: joint_jog, joint_index: 1, delta_rad: 0.1}",
                    "{capability: joint_path, waypoints_rad: []}",
                    "{capability: joint_path, waypoints_rad: [[2]]}",
                    "{capability: joint_path, waypoints_rad: [[0]], max_velocity_rad_s: 0.4}",
                    "{capability: joint_trajectory, times_s: [1, 2], positions_rad: [[0], [0]]}",
                    "{capability: joint_trajectory, times_s: [0, 0], positions_rad: [[0], [0]]}",
                    "{capability: joint_trajectory, times_s: [0, 0.01], positions_rad: [[0], [1]]}",
                    "{capability: joint_trajectory, times_s: [0, 1], positions_rad: [[0], [0]], "
                    "command_rate_hz: .nan}",
                    "{capability: arm_hold, extra: 1}", "{capability: missing}", "[]"}) {
        Reject([&] { capabilities.Parse(YAML::Load(yaml), context); },
                "invalid registered capability parameters accepted");
    }
    Reject([&] { capabilities.Validate({"joint_path", {}, {}}, context); },
            "wrong payload type accepted");
}

void SequenceChecks(const std::filesystem::path& directory) {
    auto drivers = m::MakeBuiltinDeviceRegistry();
    auto capabilities = m::MakeBuiltinCapabilityRegistry();
    m::DeviceDefinition definition;
    definition.arm = {"arm", "registered_sequence", {{"j1", -1, 1}}};
    definition.effector = {"hand", {"finger"}};
    auto devices = drivers.Create("mock_device", definition, {});
    const auto context = m::DescribeDeviceCapabilities(definition, devices);
    RegistryChecks(context);
    m::DeviceSession session(definition, std::move(devices), directory.string());
    session.Connect();

    // Existing embedded callers retain the three-field aggregate and original results.
    const std::vector<m::DeviceStep> legacy{
            {"joint_point", {{0.1}, 0.05}, {}}, {"effector_position", {}, {{0.2}, 0.1}}};
    Check(m::RunDeviceSteps(session, legacy).status == m::Result::kSucceeded,
            "legacy point/effector sequence changed");
    Check(session.ReadArm().joints_rad == std::vector<double>{0.1}, "legacy arm target lost");
    Check(session.ReadEffector().positions == std::vector<double>{0.2}, "legacy hand target lost");

    auto parse = [&](const char* yaml) { return capabilities.Parse(YAML::Load(yaml), context); };
    const char* trajectory_step =
            "{capability: joint_trajectory, times_s: [0, 0.05], positions_rad: [[0], [0.01]]}";
    const std::vector<m::DeviceStep> composed{
            parse("{capability: joint_jog, joint_index: 0, delta_rad: 0.1}"),
            parse("{capability: joint_path, waypoints_rad: [[0.1], [0.0]]}"),
            parse(trajectory_step), parse("{capability: arm_hold}"),
            parse("{capability: effector_hold}")};
    Check(m::RunDeviceSteps(session, composed).status == m::Result::kSucceeded,
            "registered motion sequence failed");
    Check(session.ReadArm().joints_rad == std::vector<double>{0.01}, "trajectory result lost");
    Reject([&] { m::RunDeviceSteps(session, {legacy.front(), {"unknown", {}, {}}}); },
            "unregistered later capability accepted");
    Check(session.ReadArm().joints_rad == std::vector<double>{0.01},
            "sequence wrote before validating later steps");
    Check(m::RunDeviceSteps(session, legacy, [] { return true; }).status == m::Result::kCancelled,
            "pre-cancellation lost");
    Check(session.ReadArm().joints_rad == std::vector<double>{0.01}, "cancelled sequence moved");
    Check(m::RunDeviceSteps(session, {}).status == m::Result::kIdle, "empty sequence not idle");
    Check(m::RunDeviceSteps(session, {}, {}).status == m::Result::kIdle,
            "legacy explicit empty cancellation callback changed");

    unsigned executions = 0;
    capabilities.Register({"probe", "Test registration without changing the runner",
            [](const YAML::Node& node) {
                return m::DeviceStep{
                        node["capability"].as<std::string>(), {}, {}, node["result"].as<int>()};
            },
            [](const m::CapabilityContext&) { return ""; },
            [](const m::DeviceStep& step, const m::CapabilityContext&) {
                Check(std::any_cast<int>(&step.parameters) != nullptr, "invalid probe payload");
            },
            [&](m::DeviceSession&, const m::DeviceStep& step, const std::function<bool()>&) {
                ++executions;
                return m::CommandResult{73,
                        static_cast<m::Result>(std::any_cast<int>(step.parameters)),
                        "probe evidence"};
            }});
    const auto failure_statuses = {m::Result::kCancelled, m::Result::kTimedOut, m::Result::kFault,
            m::Result::kStopUnknown};
    for (const auto status : failure_statuses) {
        executions = 0;
        const m::DeviceStep stop{"probe", {}, {}, static_cast<int>(status)};
        const auto outcome = m::RunDeviceSteps(session, {stop, stop}, capabilities, {});
        Check(outcome.status == status && outcome.command_id == 73 &&
                        outcome.detail == "probe evidence" && executions == 1,
                "runner replaced failure evidence or continued after failure");
    }
    const auto extension = capabilities.Parse(
            YAML::Load("{capability: probe, result: " +
                    std::to_string(static_cast<int>(m::Result::kUnverified)) + "}"),
            context);
    const auto filename = directory / "custom.yaml";
    const std::string custom_plan =
            "schema_version: 3\ndevice:\n  driver: mock_device\n  arm:\n    device_id: arm\n"
            "    resource_id: registered_sequence\n    joints:\n"
            "      - {name: j1, minimum_rad: -1, maximum_rad: 1}\nsteps:\n"
            "  - {capability: probe, result: ";
    std::ofstream(filename) << custom_plan << static_cast<int>(m::Result::kSucceeded) << "}\n";
    Reject([&] { m::LoadDevicePlan(filename.string(), drivers); },
            "unregistered extension accepted by the builtin registry");
    const auto extension_plan = m::LoadDevicePlan(filename.string(), drivers, capabilities);
    Check(m::RunDeviceSteps(session, extension_plan.steps, capabilities, {}).status ==
                    m::Result::kSucceeded,
            "registered extension did not load and execute through the public plan interface");
    Check(m::RunDeviceSteps(session, {extension, legacy.front()}, capabilities, {}).status ==
                    m::Result::kUnverified,
            "registered extension lost unverified evidence");

    session.Disconnect();
    definition.effector.feedback = m::EffectorFeedback::kNone;
    m::DeviceSession open_loop(
            definition, drivers.Create("mock_device", definition, {}), directory.string());
    open_loop.Connect();
    Check(m::RunDeviceSteps(open_loop, legacy).status == m::Result::kUnverified,
            "legacy open-loop sequence fabricated success");
    for (const auto& item : capabilities.Describe(m::DescribeSessionCapabilities(open_loop))) {
        if (item.id == "effector_hold")
            Check(!item.supported, "open-loop end-effector hold advertised as verified");
    }
}
}  // namespace

int main() {
    auto pattern = (std::filesystem::temp_directory_path() / "capability-test-XXXXXX").string();
    const char* created = mkdtemp(pattern.data());
    if (!created) return 1;
    const std::filesystem::path directory(created);
    try {
        OfflineChecks(directory);
        SequenceChecks(directory);
        std::filesystem::remove_all(directory);  // Only the owned mkdtemp test directory.
        std::cout << "registered capability contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        std::filesystem::remove_all(directory);
        return 1;
    }
}
