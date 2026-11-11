// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef RENDERER_BRIDGE_H
#define RENDERER_BRIDGE_H

#include <string>

#include "server.h"

namespace spacemit::manipulation {
// Owns graphics through the callbacks; configure, run and destroy on the main thread.
void ConfigureSimulationRenderer(
        bool viewer, const std::string& default_camera, SimulationServerOptions* options);
}  // namespace spacemit::manipulation
#endif  // RENDERER_BRIDGE_H
