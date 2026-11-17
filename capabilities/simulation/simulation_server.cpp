// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Explicit loopback-only simulation owner; no plan steps are executed on startup.
#include "server.h"

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "manipulation/device_plan.h"
#include "wire_protocol.h"
#if MANIPULATION_HAS_SIMULATION_RENDERER
#include "renderer_bridge.h"
#endif

namespace {
volatile std::sig_atomic_t interrupted = 0;

void Interrupt(int) {
    interrupted = 1;
}
}  // namespace

int main(int argc, char** argv) {
    namespace m = spacemit::manipulation;
    namespace w = m::simulation::wire;
    try {
        m::SimulationServerOptions options;
        std::string plan;
        std::string camera = "overview";
        bool viewer = false;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--viewer")
                viewer = true;
            else if ((argument == "--plan" || argument == "--port" || argument == "--camera") &&
                    index + 1 < argc) {
                const std::string value = argv[++index];
                if (argument == "--plan")
                    plan = value;
                else if (argument == "--camera")
                    camera = value;
                else {
                    if (value.empty() || value.size() > 5 ||
                            value.find_first_not_of("0123456789") != std::string::npos)
                        throw std::runtime_error("invalid simulation port");
                    options.port = static_cast<unsigned>(std::stoul(value));
                }
            } else
                throw std::runtime_error(
                        "usage: manipulation-simulation --plan FILE [--port 9100] [--viewer] "
                        "[--camera NAME]");
        }
        if (plan.empty()) throw std::runtime_error("simulation server requires --plan FILE");
        std::signal(SIGINT, Interrupt);
        std::signal(SIGTERM, Interrupt);
        options.cancelled = [] { return interrupted != 0; };
        options.ready = [](unsigned port, const std::string& epoch) {
            const auto greeting = w::Object({{"event", w::Quote("ready")},
                    {"port", std::to_string(port)}, {"epoch", w::Quote(epoch)}});
            std::cout << greeting << '\n' << std::flush;
        };
#if MANIPULATION_HAS_SIMULATION_RENDERER
#if MUJOCO_RENDER_USES_GL4ES
        // Process-local defaults only. Operators may deliberately override either value.
        setenv("LIBGL_GL", "21", 0);
        setenv("LIBGL_ES", "2", 0);
#endif
        m::ConfigureSimulationRenderer(viewer, camera, &options);
#else
        if (viewer) throw std::runtime_error("simulation viewer support not built");
#endif
        const auto registry = m::MakeBuiltinDeviceRegistry();
        return m::RunSimulationServer(m::LoadDevicePlan(plan, registry), options);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
