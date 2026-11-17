/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_render.h
 * @brief Optional snapshot-only RGB-D capture and interactive viewing.
 */
#ifndef MUJOCO_RENDER_H
#define MUJOCO_RENDER_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mujoco_sim {

class WorldSnapshot;

/** Named perspective camera. Dimensions must be 1..4096, at most 4194304 pixels. */
struct CameraSpec {
    std::string name;
    int width = 640;
    int height = 480;
    double min_depth_m = 0.01;
    double max_depth_m = 10.0;
};

/** Packed top-left RGB8 and optical Z-depth in metres; invalid depth is zero. */
struct RgbdFrame {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth_m;
    // Row-major K, with integer coordinates denoting pixel centres.
    std::array<double, 9> intrinsics{};
    // Row-major rigid transform: optical (x right, y down, z forward) to world.
    std::array<double, 16> world_from_camera{};
    std::uint64_t sample_id = 0;
    double sim_time = 0.0;
};

/**
 * Hidden-window capture; requires a display (EGL/OSMesa are not supported).
 * Build selects GLFW/native GL or X11/GLX (also compatible with gl4es on K3).
 * Construct, capture and destroy on the process main thread. No worker threads,
 * physics steps, resets or hardware access. Throws on invalid/unsupported input
 * or graphics failure. Context lifetimes are isolated; GLFW lifetime is shared.
 */
class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    RgbdFrame Capture(const WorldSnapshot& snapshot, const CameraSpec& camera);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/** A visible window is created only by explicitly constructing Viewer. */
struct ViewerOptions {
    std::string title = "MuJoCo World";
    int width = 960;
    int height = 720;
};

/** Main-thread-only, snapshot-only orbit viewer; mouse drag/scroll changes view. */
class Viewer {
public:
    explicit Viewer(const ViewerOptions& options = ViewerOptions{});
    ~Viewer();
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;
    void Draw(const WorldSnapshot& snapshot);
    bool ShouldClose() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim

#endif  // MUJOCO_RENDER_H
