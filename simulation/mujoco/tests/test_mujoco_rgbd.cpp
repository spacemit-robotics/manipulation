/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_mujoco_rgbd.cpp
 * @brief Real offscreen pixels, metric depth, optical pose and context lifetime.
 */
#include "mujoco_render.h"
#include "mujoco_world.h"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

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
    Check(rejected, "invalid camera input accepted");
}

float Centre(const mujoco_sim::RgbdFrame& frame) {
    return frame.depth_m[(frame.height / 2) * frame.width + frame.width / 2];
}

void Test(const std::string& directory) {
    mujoco_sim::World world(directory + "/rgbd.xml");
    const auto snapshot = world.Snapshot();
    mujoco_sim::Renderer renderer;
    mujoco_sim::CameraSpec camera;
    camera.name = "fixed";
    camera.width = 96;
    camera.height = 64;
    const auto frame = renderer.Capture(snapshot, camera);
    Check(frame.rgb.size() == 96 * 64 * 3 && frame.depth_m.size() == 96 * 64,
            "image dimensions wrong");
    Check(std::abs(Centre(frame) - 2.0f) < 0.002f, "metric depth plane incorrect");
    Check(frame.sample_id == world.SampleId() && frame.sim_time == 0.0 && world.Time() == 0.0,
            "capture stepped world or mismatched sample");
    Check(std::abs(frame.intrinsics[0] - 32.0 / std::tan(std::acos(-1.0) / 6.0)) < 1e-10 &&
                    frame.intrinsics[2] == 47.5 && frame.intrinsics[5] == 31.5,
            "camera intrinsics wrong");
    Check(frame.world_from_camera[0] == 1.0 && frame.world_from_camera[5] == -1.0 &&
                    frame.world_from_camera[10] == -1.0 && frame.world_from_camera[11] == 2.0,
            "optical-to-world transform wrong");
    const auto red = 3 * (19 * frame.width + 32);
    const auto blue = 3 * (44 * frame.width + 63);
    Check(frame.rgb[red] > frame.rgb[red + 2] + 50 && frame.rgb[blue + 2] > frame.rgb[blue] + 50,
            "RGB ordering or top-left image origin wrong");
    {
        mujoco_sim::Renderer second;
        Check(std::abs(Centre(second.Capture(snapshot, camera)) - 2.0f) < 0.002f,
                "second graphics context failed");
    }
    Check(std::abs(Centre(renderer.Capture(snapshot, camera)) - 2.0f) < 0.002f,
            "destroying another renderer invalidated GLFW/context");
    mujoco_sim::Viewer viewer;
    viewer.Draw(snapshot);
    Check(!viewer.ShouldClose(), "new viewer unexpectedly closed");
    Check(std::abs(Centre(renderer.Capture(snapshot, camera)) - 2.0f) < 0.002f,
            "visible/hidden context switching broke capture");
    world.Reset("moving");
    world.Step();
    camera.name = "moving";
    const auto moving = renderer.Capture(world.Snapshot(), camera);
    Check(std::abs(Centre(moving) - 1.65f) < 0.002f &&
                    std::abs(moving.world_from_camera[11] - 1.65) < 1e-10,
            "post-step camera geometry lags qpos");
    camera.max_depth_m = 1.0;
    Check(Centre(renderer.Capture(world.Snapshot(), camera)) == 0.0f,
            "out-of-range metric depth not invalidated");
    camera.name = "missing";
    Reject([&] { renderer.Capture(snapshot, camera); });
    camera.name = "ortho";
    Reject([&] { renderer.Capture(snapshot, camera); });
    camera.name = "calibrated";
    Reject([&] { renderer.Capture(snapshot, camera); });
    camera.name = "fixed";
    camera.width = 4097;
    Reject([&] { renderer.Capture(snapshot, camera); });
    camera.width = 96;
    camera.min_depth_m = -1;
    Reject([&] { renderer.Capture(snapshot, camera); });
    std::atomic<bool> wrong_thread_rejected{false};
    std::thread worker([&] {
        try {
            renderer.Capture(snapshot, camera);
        } catch (const std::logic_error&) {
            wrong_thread_rejected = true;
        }
    });
    worker.join();
    Check(wrong_thread_rejected, "worker-thread GLFW call accepted");
    camera.min_depth_m = 0.01;
    camera.max_depth_m = 10.0;
    std::atomic<bool> keep_stepping{true};
    std::atomic<int> steps{0};
    std::exception_ptr physics_error;
    std::thread physics([&] {
        try {
            while (keep_stepping) {
                world.Step();
                ++steps;
            }
        } catch (...) {
            physics_error = std::current_exception();
        }
    });
    try {
        for (int capture = 0; capture < 5; ++capture) {
            const auto old = renderer.Capture(snapshot, camera);
            Check(old.sample_id == snapshot.SampleId() && std::abs(Centre(old) - 2.0f) < 0.002f,
                    "concurrent physics changed an immutable render snapshot");
        }
    } catch (...) {
        keep_stepping = false;
        physics.join();
        throw;
    }
    keep_stepping = false;
    physics.join();
    if (physics_error) std::rethrow_exception(physics_error);
    Check(steps > 0, "physics did not progress while rendering snapshots");
}

}  // namespace

int main(int argc, char** argv) {
    try {
#if MUJOCO_RENDER_USES_GL4ES
        setenv("LIBGL_GL", "21", 0);
        setenv("LIBGL_ES", "2", 0);
#endif
        Check(argc == 2, "test fixture directory required");
        Test(argv[1]);
        std::cout << "RGB-D/viewer: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
