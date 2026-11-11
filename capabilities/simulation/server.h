// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Local simulation server. Render callbacks run exclusively on the caller/main thread.
#ifndef SERVER_H
#define SERVER_H

#include <functional>
#include <string>

namespace spacemit::manipulation {
struct DevicePlan;
class MujocoRuntime;

struct SimulationCaptureRequest {
    std::string camera;
    int width = 640;
    int height = 480;
    double min_depth_m = 0.01;
    double max_depth_m = 10;
};

struct SimulationCaptureReply {
    // JSON object: camera,width,height,sample_id,sim_time,K,T_world_camera,
    // rgb_bytes,depth_bytes; formats RGB8(top-left), float32 little-endian metres.
    std::string metadata;
    // One binary frame after the JSON reply: packed RGB followed by packed depth.
    std::string payload;
};

struct SimulationServerOptions {
    unsigned port = 9100;  // Zero permits an ephemeral loopback port, useful in tests.
    double lease_timeout_s = 1;
    std::function<bool()> cancelled;
    std::function<void(unsigned, const std::string&)> ready;
    // False requests an orderly stop; no live engine pointers escape the runtime.
    std::function<bool(MujocoRuntime&)> tick;
    std::function<SimulationCaptureReply(MujocoRuntime&, const SimulationCaptureRequest&)> capture;
};

// The plan must use local mujoco for every declared role. World lifetime is server lifetime,
// not connection lifetime. All listener sockets bind IPv4 loopback; SSH forwards are supported.
int RunSimulationServer(const DevicePlan& plan, const SimulationServerOptions& options);
}  // namespace spacemit::manipulation
#endif  // SERVER_H
