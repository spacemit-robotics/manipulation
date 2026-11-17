// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "observer_client.h"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "wire_protocol.h"

namespace spacemit::manipulation {
namespace {
namespace w = simulation::wire;

struct Socket {
    int descriptor = -1;

    ~Socket() {
        if (descriptor >= 0) ::close(descriptor);
    }
};

YAML::Node Reply(int socket, std::uint64_t id, Clock::time_point deadline,
        const std::string& epoch = {}, std::string* text = nullptr) {
    const auto bytes = w::ReadFrame(socket, deadline);
    const auto response = w::Parse(bytes);
    if (response["v"].as<int>() != 1 || w::Unsigned(response["id"]) != id ||
            (!epoch.empty() && response["epoch"].as<std::string>() != epoch)) {
        throw std::runtime_error("simulation camera response identity mismatch");
    }
    if (!response["ok"].as<bool>()) {
        throw std::runtime_error("simulation camera request rejected: " +
                response["error"].as<std::string>("server rejected request"));
    }
    if (text) *text = bytes;
    return response;
}
}  // namespace

SimulationCameraImage CaptureSimulationCamera(const std::string& host, unsigned port,
        const SimulationCaptureRequest& request, double timeout_s) {
    if (!std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 30 || port == 0 ||
            port > 65535 || request.width < 1 || request.width > 4096 || request.height < 1 ||
            request.height > 4096 ||
            static_cast<std::size_t>(request.width) * request.height > 4194304 ||
            request.camera.size() > 200 || request.camera.find('\0') != std::string::npos ||
            !std::isfinite(request.min_depth_m) || !std::isfinite(request.max_depth_m) ||
            request.min_depth_m <= 0 || request.max_depth_m <= request.min_depth_m) {
        throw std::invalid_argument("invalid simulation camera request");
    }
    const auto deadline = Clock::now() +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s));
    simulation::TransportOptions transport;
    transport.host = host;
    transport.port = port;
    Socket socket{w::ConnectLoopback(transport, deadline)};
    w::WriteFrame(socket.descriptor,
            w::Object({{"v", "1"}, {"id", "1"}, {"op", "\"hello\""}, {"role", "\"observer\""}}),
            deadline);
    const auto greeting = Reply(socket.descriptor, 1, deadline);
    const auto epoch = greeting["epoch"].as<std::string>();
    if (epoch.empty() || epoch.size() > 200 ||
            greeting["data"]["role"].as<std::string>() != "observer") {
        throw std::runtime_error("invalid simulation observer greeting");
    }
    w::WriteFrame(socket.descriptor,
            w::Object({{"v", "1"}, {"id", "2"}, {"op", "\"capture\""}, {"epoch", w::Quote(epoch)},
                    {"camera", w::Quote(request.camera)}, {"width", std::to_string(request.width)},
                    {"height", std::to_string(request.height)},
                    {"min_depth_m", w::Number(request.min_depth_m)},
                    {"max_depth_m", w::Number(request.max_depth_m)}}),
            deadline);
    std::string metadata;
    const auto response = Reply(socket.descriptor, 2, deadline, epoch, &metadata);
    const auto data = response["data"];
    w::Keys(data,
            {"camera", "width", "height", "sample_id", "sim_time", "K", "T_world_camera",
                    "rgb_bytes", "depth_bytes"});
    const auto pixels = static_cast<std::size_t>(request.width) * request.height;
    if (data["width"].as<int>() != request.width || data["height"].as<int>() != request.height ||
            w::Unsigned(data["rgb_bytes"]) != pixels * 3 ||
            w::Unsigned(data["depth_bytes"]) != pixels * 4 || w::Finite(data["sim_time"]) < 0 ||
            (!request.camera.empty() && data["camera"].as<std::string>() != request.camera)) {
        throw std::runtime_error("simulation image metadata mismatch");
    }
    w::Unsigned(data["sample_id"]);
    w::Numbers(data["K"], 9);
    w::Numbers(data["T_world_camera"], 16);
    const auto bytes = w::ReadFrame(socket.descriptor, deadline, w::kImageLimit);
    if (bytes.size() != pixels * 7) throw std::runtime_error("simulation image size mismatch");
    for (std::size_t offset = pixels * 3; offset < bytes.size(); offset += 4) {
        std::uint32_t bits = 0;
        for (unsigned index = 0; index < 4; ++index) {
            bits |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset + index]))
                    << (8 * index);
        }
        float depth;
        std::memcpy(&depth, &bits, sizeof(depth));
        if (!std::isfinite(depth) || depth < 0) {
            throw std::runtime_error("invalid simulation image depth");
        }
    }
    return {request.width, request.height, metadata, bytes.substr(0, pixels * 3),
            bytes.substr(pixels * 3)};
}
}  // namespace spacemit::manipulation
