/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_mujoco_snapshot.cpp
 * @brief Snapshot ownership, refreshed geometry, contacts and atomic weld edits.
 */
#include "mujoco_native_snapshot.h"
#include "mujoco_world.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Reject(const std::function<void()>& call) {
    bool rejected = false;
    try {
        call();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Check(rejected, "invalid operation accepted");
}

int control_calls = 0;

void CountControl(const mjModel*, mjData*) {
    ++control_calls;
}

std::size_t allocation_count = 0;
std::size_t allocated_bytes = 0;

void* CountAllocation(std::size_t bytes) {
    ++allocation_count;
    allocated_bytes += bytes;
    void* allocation = nullptr;
    if (posix_memalign(&allocation, 64, bytes) != 0) return nullptr;
    return allocation;
}

std::pair<std::size_t, std::size_t> MeasureAllocations(const std::function<void()>& call) {
    Check(mju_user_malloc == nullptr, "unexpected custom MuJoCo allocator");
    allocation_count = 0;
    allocated_bytes = 0;
    mju_user_malloc = CountAllocation;
    try {
        call();
    } catch (...) {
        mju_user_malloc = nullptr;
        throw;
    }
    mju_user_malloc = nullptr;
    return {allocation_count, allocated_bytes};
}

void TestOwnerAllocations(const std::string& directory) {
    mujoco_sim::World world(directory + "/snapshot.xml");
    mujoco_sim::WorldSnapshot before;
    const auto baseline = MeasureAllocations([&] { before = world.Snapshot(); });
    Check(baseline.first > 0, "allocation counter did not observe snapshot data copy");
    mujoco_sim::NativeSnapshot inspection(before);
    const auto* native_model = inspection.Model();
    const auto native_baseline = MeasureAllocations([&] { inspection.Update(before); });
    mjcb_control = CountControl;
    control_calls = 0;
    const auto edit_allocations = MeasureAllocations([&] {
        Check(world.TouchingBodies("anchor", "object"), "first owner contact missing");
        Check(world.TouchingBodies("pad", "object", false), "second owner contact missing");
        world.AttachWeld("attachment", "anchor", "object");
    });
    mjcb_control = nullptr;
    Check(edit_allocations.first == 0, "owner contact/weld path allocated MuJoCo model/data");
    Check(control_calls == 0, "owner geometry refresh called controller");
    mujoco_sim::WorldSnapshot attached;
    const auto edited = MeasureAllocations([&] { attached = world.Snapshot(); });
    Check(edited == baseline, "weld edit caused a full model clone on the snapshot path");
    const auto native_edited = MeasureAllocations([&] { inspection.Update(attached); });
    Check(native_edited == native_baseline && inspection.Model() == native_model,
            "weld edit caused a full model clone on the native inspection path");
    const int weld = mj_name2id(inspection.Model(), mjOBJ_EQUALITY, "attachment");
    const int object = mj_name2id(inspection.Model(), mjOBJ_BODY, "object");
    Check(inspection.Model()->eq_obj2id[weld] == object,
            "reused native model missed updated equality target");
    inspection.Update(before);
    Check(inspection.Model()->eq_obj2id[weld] != object,
            "reused native model failed to restore old snapshot equality target");
    world.Reset();
    mujoco_sim::WorldSnapshot reset;
    Check(MeasureAllocations([&] { reset = world.Snapshot(); }) == baseline,
            "reset caused a full model clone on the snapshot path");
    world.Reset("moving");
    world.TouchingBodies("anchor", "object");
    world.Step();
    world.AttachWeld("attachment", "anchor", "camera_carrier");
    inspection.Update(world.Snapshot());
    Check(std::abs(inspection.Model()->eq_data[weld * mjNEQDATA + 5] - 0.61) < 1e-10,
            "owner geometry cache did not refresh after integration");
}

void Test(const std::string& directory) {
    mujoco_sim::WorldSnapshot retained;
    Reject([&] { retained.Time(); });
    {
        mujoco_sim::World world(directory + "/snapshot.xml");
        world.Reset("moving");
        retained = world.Snapshot();
        world.Step();
        const auto current = world.Snapshot();
        mujoco_sim::NativeSnapshot previous_native(retained);
        const auto* reused_model = previous_native.Model();
        mjcb_control = CountControl;
        const mujoco_sim::NativeSnapshot native(current);
        mjcb_control = nullptr;
        Check(control_calls == 0, "snapshot ran control callback");
        Check(native.Model() != previous_native.Model(), "native model copies are shared");
        previous_native.Update(current);
        Check(previous_native.Model() == reused_model, "model cloned for every update");
        const int camera = mj_name2id(native.Model(), mjOBJ_CAMERA, "moving");
        Check(std::abs(native.Data()->cam_xpos[3 * camera + 2] - 1.61) < 1e-10,
                "camera geometry does not match post-step qpos");
        Check(retained.Joint("camera_height").position == 0.0,
                "stepping changed retained snapshot");
        Check(current.SampleId() == world.SampleId() && current.Time() == world.Time(),
                "snapshot timestamp mismatch");
        world.Reset();
        Check(world.TouchingBodies("anchor", "object"), "descendant contact missing");
        Check(!world.TouchingBodies("anchor", "object", false), "direct body filter ignored");
        Check(world.TouchingBodies("pad", "object", false), "pad contact missing");
        Check(!world.TouchingBodies("anchor", "other"), "non-contact reported touching");
        const auto before = world.Snapshot();
        const mujoco_sim::NativeSnapshot before_native(before);
        const int weld = mj_name2id(before_native.Model(), mjOBJ_EQUALITY, "attachment");
        Reject([&] { world.AttachWeld("attachment", "object", "object"); });
        Reject([&] { world.AttachWeld("attachment", "anchor", "missing"); });
        Reject([&] { world.ReleaseWeld("missing"); });
        Check(world.SampleId() == before.SampleId(), "invalid edit created sample");
        world.AttachWeld("attachment", "anchor", "object");
        const auto attached = world.Snapshot();
        const mujoco_sim::NativeSnapshot attached_native(attached);
        Check(attached.SampleId() == before.SampleId() + 1 && attached.Time() == before.Time(),
                "attach advanced physics or missed sample");
        Check(attached_native.Model() != before_native.Model(), "model revision not isolated");
        Check(attached_native.Data()->eq_active[weld] == 1 &&
                        before_native.Data()->eq_active[weld] == 0,
                "weld active state leaked into old snapshot");
        const int object = mj_name2id(attached_native.Model(), mjOBJ_BODY, "object");
        Check(attached_native.Model()->eq_obj2id[weld] == object &&
                        before_native.Model()->eq_obj2id[weld] != object,
                "weld target/model snapshot incorrect");
        Check(std::abs(attached_native.Model()->eq_data[weld * mjNEQDATA + 3] - 0.15) < 1e-10,
                "weld relative pose incorrect");
        world.ReleaseWeld("attachment");
        // C structs are shallow-const: accidental writes must stay detached.
        before_native.Model()->eq_obj2id[weld] = object;
        const mujoco_sim::NativeSnapshot isolated(before);
        Check(isolated.Model()->eq_obj2id[weld] != object,
                "native inspection leaked into immutable model snapshots");
        const mujoco_sim::NativeSnapshot released(world.Snapshot());
        Check(released.Data()->eq_active[weld] == 0, "release left weld active");
        world.Reset();
        const mujoco_sim::NativeSnapshot reset(world.Snapshot());
        Check(reset.Model()->eq_obj2id[weld] == isolated.Model()->eq_obj2id[weld] &&
                        reset.Data()->eq_active[weld] == 0,
                "reset did not restore XML equality settings");
    }
    Check(retained.IsValid() && retained.Time() == 0.0, "snapshot died with world");
    const mujoco_sim::NativeSnapshot surviving(retained);
    Check(surviving.Model()->nbody > 0, "native inspection lost model ownership");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 2, "test fixture directory required");
        Test(argv[1]);
        TestOwnerAllocations(argv[1]);
        std::cout << "snapshot/weld: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        mjcb_control = nullptr;
        std::cerr << error.what() << '\n';
        return 1;
    }
}
