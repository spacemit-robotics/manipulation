// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "renderer_bridge.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "manipulation/mujoco_runtime.h"
#include "mujoco_render.h"
#include "mujoco_world.h"
#include "wire_protocol.h"

namespace spacemit::manipulation {
namespace {
namespace w = simulation::wire;

struct Graphics {
    std::unique_ptr<mujoco_sim::Renderer> renderer;
    std::unique_ptr<mujoco_sim::Viewer> viewer;
    Clock::time_point next_draw{};
};

std::string Encode(const mujoco_sim::RgbdFrame& frame) {
    const auto pixels = static_cast<std::size_t>(frame.width) * frame.height;
    if (frame.rgb.size() != pixels * 3 || frame.depth_m.size() != pixels ||
            pixels > w::kImageLimit / 7) {
        throw std::runtime_error("invalid simulation image size");
    }
    std::string bytes(frame.rgb.begin(), frame.rgb.end());
    bytes.reserve(pixels * 7);
    for (float depth : frame.depth_m) {
        static_assert(sizeof(float) == 4, "camera wire format requires float32");
        std::uint32_t bits;
        std::memcpy(&bits, &depth, sizeof(bits));
        for (unsigned shift = 0; shift < 32; shift += 8) {
            bytes.push_back(static_cast<char>((bits >> shift) & 255));
        }
    }
    return bytes;
}
}  // namespace

void ConfigureSimulationRenderer(
        bool viewer, const std::string& default_camera, SimulationServerOptions* options) {
    if (!options) throw std::invalid_argument("missing simulation server options");
    auto graphics = std::make_shared<Graphics>();
    if (viewer) {
        graphics->viewer = std::make_unique<mujoco_sim::Viewer>();
        options->initialize = [graphics](MujocoRuntime& runtime) {
            graphics->viewer->Draw(runtime.Snapshot());
            graphics->next_draw = Clock::now() + std::chrono::milliseconds(33);
        };
    }
    // A camera context is created only when the first observer requests an image.
    options->tick = [graphics](MujocoRuntime& runtime) {
        if (!graphics->viewer) return true;
        if (graphics->viewer->ShouldClose()) return false;
        if (Clock::now() >= graphics->next_draw) {
            graphics->viewer->Draw(runtime.Snapshot());
            graphics->next_draw = Clock::now() + std::chrono::milliseconds(33);
        }
        return !graphics->viewer->ShouldClose();
    };
    options->capture = [graphics, default_camera](auto& runtime, const auto& request) {
        if (!graphics->renderer) graphics->renderer = std::make_unique<mujoco_sim::Renderer>();
        const auto name = request.camera.empty() ? default_camera : request.camera;
        const auto frame = graphics->renderer->Capture(runtime.Snapshot(),
                {name, request.width, request.height, request.min_depth_m, request.max_depth_m});
        SimulationCaptureReply result;
        result.payload = Encode(frame);
        result.metadata = w::Object({{"camera", w::Quote(name)},
                {"width", std::to_string(frame.width)}, {"height", std::to_string(frame.height)},
                {"sample_id", std::to_string(frame.sample_id)},
                {"sim_time", w::Number(frame.sim_time)},
                {"K", w::Array({frame.intrinsics.begin(), frame.intrinsics.end()})},
                {"T_world_camera",
                        w::Array({frame.world_from_camera.begin(), frame.world_from_camera.end()})},
                {"rgb_bytes", std::to_string(frame.rgb.size())},
                {"depth_bytes", std::to_string(frame.depth_m.size() * sizeof(float))}});
        return result;
    };
}
}  // namespace spacemit::manipulation
