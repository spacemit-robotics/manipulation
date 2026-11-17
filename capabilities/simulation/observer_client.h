// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef OBSERVER_CLIENT_H
#define OBSERVER_CLIENT_H

#include <string>

#include "server.h"

namespace spacemit::manipulation {
struct SimulationCameraImage {
    int width = 0;
    int height = 0;
    std::string metadata;  // JSON includes server epoch, K and optical camera transform.
    std::string rgb;       // Packed RGB8, top-left origin.
    std::string depth;     // Packed float32 little-endian optical Z in metres, top-left origin.
};

// Observer role only: no controller lease, movement, reset or reconnection/replay.
SimulationCameraImage CaptureSimulationCamera(const std::string& host, unsigned port,
        const SimulationCaptureRequest& request, double timeout_s = 10);
}  // namespace spacemit::manipulation
#endif  // OBSERVER_CLIENT_H
