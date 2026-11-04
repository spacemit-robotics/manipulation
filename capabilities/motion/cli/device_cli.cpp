// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Public device/effector tools. No startup motion; `serve --read-only` for tools that only read.
#include "manipulation/device_plan.h"
#include "arm_check.h"
#include "device_service.h"
#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
static_assert(std::atomic<bool>::is_always_lock_free, "signal flag must be lock-free");
std::atomic<bool> interrupted{false};
void Signal(int) { interrupted.store(true, std::memory_order_relaxed); }
void Print(spacemit::manipulation::DeviceSession& session) {
    const auto arm = session.ReadArm();
    std::cout << std::setprecision(12) << "{\"firmware\":" << JsonString(arm.firmware)
        << ",\"enabled\":" << arm.enabled << ",\"position_mode\":" << arm.position_mode
        << ",\"moving\":" << arm.moving << ",\"fault\":" << arm.fault << ",\"joints_rad\":[";
    for (std::size_t i = 0; i < arm.joints_rad.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << arm.joints_rad[i];
    }
    std::cout << ']';
    if (session.HasEffector()) {
        const auto hand = session.ReadEffector();
        const bool measured = session.Definition().effector.feedback == spacemit::manipulation::EffectorFeedback::kPosition;
        std::cout << ",\"effector_feedback\":\"" << (measured ? "position" : "none") << "\",\"effector_positions\":[";
        for (std::size_t i = 0; i < hand.positions.size(); ++i) {
            if (i) std::cout << ',';
            std::cout << hand.positions[i];
        }
        std::cout << "],\"effector_fault\":" << hand.fault << ",\"effector_enabled\":"
            << (hand.enabled.has_value() ? (*hand.enabled ? "true" : "false") : "null") << ",\"effector_loads\":[";
        for (std::size_t i = 0; i < hand.loads.size(); ++i) std::cout << (i ? "," : "") << hand.loads[i];
        std::cout << "],\"effector_holding\":"
            << (hand.holding.has_value() ? (*hand.holding ? "true" : "false") : "null");
    }
    std::cout << "}\n";
}
}  // namespace
int main(int argc, char** argv) {
    namespace m = spacemit::manipulation;
    try {
        auto registry = m::MakeBuiltinDeviceRegistry();
        if (argc == 2 && std::string(argv[1]) == "drivers") {
            for (const auto& id : registry.List()) std::cout << id << '\n';
            return 0;
        }
        const std::string usage =
            "manipulation-device drivers | validate PLAN | describe PLAN | capabilities PLAN | inspect PLAN |\n"
            "    run PLAN [--enable] | serve PLAN [--read-only | --enable | --allow-disable |\n"
            "    --allow-effector-enable | --allow-effector-disable] | arm-check PLAN [--timeout-only]\n"
            "Device leases are host-wide in " + std::string(m::DefaultLockDirectory()) + ".";
        if (argc < 3) throw std::runtime_error(usage);
        const std::string operation = argv[1];
        const std::vector<std::string> flags(argv + 3, argv + argc);
        const auto has = [&flags](const char* flag) {
            for (const auto& item : flags) if (item == flag) return true;
            return false;
        };
        for (const auto& item : flags) {
            if (item != "--read-only" && item != "--enable" && item != "--timeout-only" &&
                item != "--allow-disable" && item != "--allow-effector-enable" &&
                item != "--allow-effector-disable") {
                throw std::runtime_error("unknown option " + item + "\n" + usage);
            }
        }
        const bool read_only = has("--read-only"), enable = has("--enable"), timeout_only = has("--timeout-only");
        const bool disable = has("--allow-disable"), effector_enable = has("--allow-effector-enable"),
            effector_disable = has("--allow-effector-disable");
        // Read-only service for tools that must never move the device; every motion is refused.
        if (read_only && (operation != "serve" || enable || disable || effector_enable || effector_disable)) {
            throw std::runtime_error("--read-only is a serve option and excludes torque permissions");
        }
        if ((disable || effector_enable || effector_disable) && operation != "serve") {
            throw std::runtime_error("torque permission flags require serve");
        }
        if (timeout_only && operation != "arm-check") throw std::runtime_error("--timeout-only is an arm-check option");
        if ((operation == "validate" || operation == "inspect" || operation == "describe" ||
            operation == "capabilities") && !flags.empty()) {
            throw std::runtime_error(usage);
        }
        if (operation == "arm-check" && enable) throw std::runtime_error("arm-check never enables the arm");
        if (operation != "validate" && operation != "describe" && operation != "inspect" && operation != "run" &&
            operation != "serve" && operation != "arm-check" && operation != "capabilities") {
            throw std::runtime_error(usage);
        }
        const auto plan = m::LoadDevicePlan(argv[2], registry);
        if (operation == "validate") { std::cout << "valid\n"; return 0; }
        if (operation == "describe") {  // Offline metadata for tools; never opens the device.
            std::cout << "{\"driver\":" << JsonString(plan.driver) << ",\"model\":" << JsonString(plan.model)
                << ",\"joints\":[";
            for (std::size_t i = 0; i < plan.definition.arm.joints.size(); ++i) {
                std::cout << (i ? "," : "") << JsonString(plan.definition.arm.joints[i].name);
            }
            std::cout << "],\"joint_limits_rad\":[";
            for (std::size_t i = 0; i < plan.definition.arm.joints.size(); ++i) {
                const auto& joint = plan.definition.arm.joints[i];
                std::cout << (i ? "," : "") << std::setprecision(17) << '[' << joint.minimum_rad << ','
                    << joint.maximum_rad << ']';
            }
            std::cout << "],\"arm_timeout_s\":" << plan.definition.arm.timeout_s << ",\"channels\":[";
            for (std::size_t i = 0; i < plan.definition.effector.channels.size(); ++i) {
                std::cout << (i ? "," : "") << JsonString(plan.definition.effector.channels[i]);
            }
            std::cout << "],\"effector_feedback\":" << JsonString(
                plan.definition.effector.channels.empty() ? "" :
                plan.definition.effector.feedback == m::EffectorFeedback::kNone ? "none" : "position") << "}\n";
            return 0;
        }
        std::signal(SIGINT, Signal);
        std::signal(SIGTERM, Signal);
        std::signal(SIGPIPE, SIG_IGN);  // Preserve cancellation/destruction when a local client exits.
        if (operation == "arm-check") {
            return m::RunArmCheck(plan, registry, timeout_only, [] { return interrupted != 0; }, std::cout);
        }
        m::DeviceSession session(plan.definition, registry.Create(plan.driver, plan.definition, plan.options));
        if (operation == "capabilities") {
            std::cout << DeviceCapabilitiesJson(session) << '\n';
            return 0;
        }
        session.Connect();
        session.Diagnose();
        if (operation == "serve") {
            return ServeDevice(session, !read_only, enable, [] { return interrupted != 0; }, plan.driver,
                disable, effector_enable, effector_disable);
        }
        Print(session);
        if (operation == "inspect") return 0;
        if (interrupted) return 1;
        if (enable) session.EnableArm();
        const auto result = m::RunDeviceSteps(session, plan.steps, [] { return interrupted != 0; });
        std::cout << "{\"status\":\"" << m::ResultName(result.status)
            << "\",\"physical_grip_verified\":false}\n";
        // 3: every step was delivered, but open-loop effector steps were not measured.
        if (result.status == m::Result::kUnverified) return 3;
        return result.status == m::Result::kSucceeded ? 0 : 1;
    } catch (const std::exception& error) {
        // Startup can fail before ServeDevice sends its ready frame. Keep the cause available to
        // local clients instead of making them infer it from stderr or a subsequent EOF.
        if (argc > 1 && std::string(argv[1]) == "serve") {
            std::cout << "{\"v\":1,\"id\":0,\"ok\":false,\"error\":"
                << JsonString(error.what()) << "}\n" << std::flush;
        }
        std::cerr << error.what() << '\n';
        return 2;
    }
}
