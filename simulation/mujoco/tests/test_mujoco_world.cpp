/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_mujoco_world.cpp
 * @brief Headless world mapping, atomic controls, independent state and faults.
 */
#include "mujoco_world.h"

#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void Check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool Near(double actual, double expected) {
    return std::abs(actual - expected) < 1e-10;
}

template <typename Exception>
void ExpectFailure(const std::function<void()>& operation, const char* message) {
    bool failed = false;
    try {
        operation();
    } catch (const Exception&) {
        failed = true;
    }
    Check(failed, message);
}

void TestMappingAndReset(const std::string& model_path) {
    const auto cwd = std::filesystem::current_path();
    mujoco_sim::World world(model_path);
    mujoco_sim::World independent(model_path);
    const auto relative_path = std::filesystem::relative(model_path, cwd);
    mujoco_sim::World relative(relative_path.string());
    Check(std::filesystem::current_path() == cwd, "loading changed cwd");
    Check(Near(relative.Time(), 0.0), "relative model loading failed");
    Check(world.SampleId() == 1 && Near(world.Time(), 0.0), "invalid initial sample");
    Check(Near(world.Timestep(), 0.002), "model timestep was changed");
    const auto arm = world.Actuator("arm_position");
    Check(arm.position_control && arm.affine_position_control && arm.joint_transmission,
            "position servo metadata missing");
    Check(arm.joint_name == "axis" && Near(arm.gear, 1.0), "direct transmission misreported");
    Check(arm.transmission_supported && arm.enabled && Near(arm.control, 0.0) &&
                    arm.controlled_joints == std::vector<std::string>{"axis"},
            "direct controlled-joint metadata missing");
    Check(arm.control_limited && Near(arm.minimum, -1.0) && Near(arm.maximum, 1.0),
            "control limits misreported");
    const auto gripper = world.Actuator("gripper");
    Check(gripper.affine_position_control && !gripper.position_control &&
                    !gripper.joint_transmission,
            "scaled tendon servo misreported");
    Check(gripper.transmission_supported && gripper.enabled && Near(gripper.gear, 1.0) &&
                    gripper.controlled_joints == std::vector<std::string>{"opening"},
            "fixed tendon joint mapping misreported");
    Check(!world.Actuator("runaway").position_control, "motor mistaken for position servo");
    Check(Near(world.Actuator("geared").gear, 2.0), "nonunit gear was hidden");
    Check(world.Actuator("geared").joint_name == "auxiliary", "wrong joint mapping");
    world.Reset("posed");
    Check(Near(world.Actuator("arm_position").control, 0.3) &&
                    Near(world.Actuator("gripper").control, 0.08),
            "keyframe actuator controls were hidden");
    const auto hinge = world.Joint("axis");
    const auto slide = world.Joint("opening");
    Check(hinge.type == mujoco_sim::JointType::kHinge && Near(hinge.position, 0.3) &&
                    Near(hinge.velocity, 0.1),
            "hinge qpos/qvel mapping ignored preceding free joint");
    Check(slide.type == mujoco_sim::JointType::kSlide && Near(slide.position, 0.04) &&
                    Near(slide.velocity, 0.3),
            "slide qpos/qvel mapping ignored passive joint");
    Check(hinge.limited && Near(hinge.minimum, -1.0) && Near(hinge.maximum, 1.0),
            "joint limits misreported");
    Check(Near(world.Time(), 0.5) && world.SampleId() == 2, "keyframe time/sample invalid");
    Check(Near(independent.Joint("axis").position, 0.0), "worlds share mutable data");
    ExpectFailure<std::invalid_argument>(
            [&]() { world.Reset("missing"); }, "unknown keyframe accepted");
    Check(Near(world.Time(), 0.5) && world.SampleId() == 2, "failed reset changed state");
    ExpectFailure<std::invalid_argument>(
            [&]() { world.Joint("object_free"); }, "free joint was exposed as a scalar");
    ExpectFailure<std::invalid_argument>(
            [&]() { world.Joint("missing"); }, "unknown joint accepted");
    ExpectFailure<std::invalid_argument>(
            [&]() { world.Actuator(""); }, "empty actuator name accepted");
    world.Reset();
    Check(Near(world.Time(), 0.0) && Near(world.Joint("axis").position, 0.0),
            "default reset did not restore model defaults");
    world.Step(0);
    Check(world.SampleId() == 3, "zero steps created a sample");
    world.SetControls({{"arm_position", 0.4}, {"gripper", 0.2}});
    Check(Near(world.Actuator("arm_position").control, 0.4) &&
                    Near(world.Actuator("gripper").control, 0.2),
            "current actuator controls did not reflect writes");
    world.Step(100);
    Check(world.Joint("axis").position > 0.2 && world.Joint("opening").position > 0.0,
            "named arm/tendon actuators did not drive their joints");
    Check(Near(world.Time(), 0.2) && world.SampleId() == 103, "step accounting incorrect");
    Check(Near(independent.Time(), 0.0), "stepping advanced another world");
    Check(std::filesystem::current_path() == cwd, "world operations changed cwd");
}

void TestAtomicControls(const std::string& model_path) {
    mujoco_sim::World world(model_path);
    mujoco_sim::World reference(model_path);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::vector<std::vector<std::pair<std::string, double>>> invalid_batches = {
            {{"arm_position", 0.5}, {"missing", 0.0}},
            {{"arm_position", 0.5}, {"gripper", 0.5}},
            {{"arm_position", 0.5}, {"gripper", nan}},
            {{"arm_position", 0.5}, {"runaway", infinity}},
            {{"arm_position", 0.5}, {"arm_position", 0.2}},
    };
    for (const auto& batch : invalid_batches) {
        world.Reset();
        reference.Reset();
        world.SetControls({{"arm_position", -0.3}});
        reference.SetControls({{"arm_position", -0.3}});
        ExpectFailure<std::invalid_argument>(
                [&]() { world.SetControls(batch); }, "invalid control batch accepted");
        world.SetControls({});
        world.Step(20);
        reference.Step(20);
        Check(Near(world.Joint("axis").position, reference.Joint("axis").position),
                "rejected batch partially changed controls");
        Check(Near(world.Joint("opening").position, reference.Joint("opening").position),
                "rejected batch changed tendon control");
    }
    world.SetControls({{"arm_position", -1.0}, {"gripper", 0.4}});
    world.SetControls({{"arm_position", 1.0}, {"gripper", -0.4}});
    world.Reset();
    reference.Reset();
    world.SetControls({{"gripper", 0.2}});
    world.SetControls({{"arm_position", 0.1}});
    reference.SetControls({{"gripper", 0.2}, {"arm_position", 0.1}});
    world.Step(20);
    reference.Step(20);
    Check(Near(world.Joint("opening").position, reference.Joint("opening").position),
            "partial control batch cleared an unmentioned actuator");
}

void TestInvalidPhysics(const std::string& model_path) {
    mujoco_sim::World world(model_path);
    world.SetControls({{"runaway", 1e30}});
    ExpectFailure<std::runtime_error>(
            [&]() { world.Step(10); }, "unstable physics silently continued after reset/warning");
    ExpectFailure<std::runtime_error>(
            [&]() { world.Joint("axis"); }, "faulted world published successful joint state");
    ExpectFailure<std::runtime_error>(
            [&]() { world.Time(); }, "faulted world published successful time");
    ExpectFailure<std::runtime_error>(
            [&]() { world.SampleId(); }, "faulted world published successful sample");
    ExpectFailure<std::runtime_error>(
            [&]() { world.Snapshot(); }, "faulted world published a successful snapshot");
    ExpectFailure<std::runtime_error>(
            [&]() { world.SetControls({}); }, "faulted world accepted controls before reset");
    world.Reset();
    world.Step();
    Check(Near(world.Time(), world.Timestep()), "explicit reset did not recover fault");
}

void TestTransmissionMetadata(const std::string& directory) {
    mujoco_sim::World world(directory + "/world_transmission.xml");
    const auto split = world.Actuator("split_control");
    const std::vector<std::string> expected = {"left", "right"};
    Check(split.transmission_supported && split.controlled_joints == expected &&
                    Near(split.gear, 2.0),
            "multi-joint fixed tendon or its gear was misreported");
    const std::vector<std::string> unsupported = {
            "zero_control", "spatial_control", "ball_control", "zero_gear", "site_control"};
    for (const auto& name : unsupported) {
        const auto actuator = world.Actuator(name);
        Check(!actuator.transmission_supported && actuator.controlled_joints.empty(),
                "unsupported transmission advertised controlled scalar joints");
    }
}

void TestDisabledActuators(const std::string& directory) {
    mujoco_sim::World global(directory + "/world_disabled_global.xml");
    const std::vector<std::string> all_actuators = {"arm_position", "gripper", "runaway", "geared"};
    for (const auto& name : all_actuators) {
        Check(!global.Actuator(name).enabled, "global actuation disable flag ignored");
    }
    global.SetControls({{"arm_position", 0.2}, {"gripper", 0.2}});
    global.Step(20);
    Check(Near(global.Joint("axis").position, 0.0), "disabled actuator moved its joint");
    mujoco_sim::World grouped(directory + "/world_disabled_group.xml");
    Check(grouped.Actuator("arm_position").enabled && grouped.Actuator("gripper").enabled,
            "unmasked actuator group was marked disabled");
    Check(!grouped.Actuator("runaway").enabled && !grouped.Actuator("geared").enabled,
            "disabled actuator group was marked enabled");
    grouped.SetControls({{"runaway", 100.0}, {"arm_position", 0.2}});
    grouped.Step(20);
    Check(Near(grouped.Joint("auxiliary").position, 0.0) && grouped.Joint("axis").position > 0.0,
            "actuator group metadata disagrees with physical motion");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "expected test fixture directory");
        const std::string model_path = std::string(argv[1]) + "/world.xml";
        ExpectFailure<std::invalid_argument>(
                []() { mujoco_sim::World world(""); }, "empty XML path accepted");
        ExpectFailure<std::runtime_error>(
                [&]() { mujoco_sim::World world(model_path + ".missing"); },
                "missing XML file accepted");
        ExpectFailure<std::runtime_error>(
                [&]() { mujoco_sim::World world(std::string(argv[1]) + "/world_invalid.xml"); },
                "invalid model timestep accepted");
        TestMappingAndReset(model_path);
        TestAtomicControls(model_path);
        TestInvalidPhysics(model_path);
        TestTransmissionMetadata(argv[1]);
        TestDisabledActuators(argv[1]);
        std::cout << "headless MuJoCo world tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
