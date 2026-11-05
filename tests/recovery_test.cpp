// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Test-only caller of the public DeviceSession API. Never installed or used on real hardware.
#include "manipulation/device_plan.h"

#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace m = spacemit::manipulation;

namespace {
void Check(bool condition, const char* detail) {
    if (!condition) throw std::runtime_error(detail);
}

void Reject(const std::function<void()>& operation) {
    bool rejected = false;
    try {
        operation();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, "faulted session accepted a new operation");
}

void Control(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream stream(path);
    stream << contents;
    stream.close();
    Check(!stream.fail(), "cannot update test-only control file");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Check(argc == 3, "test requires PLAN SCENARIO");
        const std::string scenario = argv[2];
        Check(scenario == "worker_exit" || scenario == "rpc_timeout" || scenario == "truncated" ||
                      scenario == "fault" || scenario == "motor" || scenario == "disabled" ||
                      scenario == "mode" || scenario == "stale" || scenario == "part",
              "unknown test scenario");
        auto registry = m::MakeBuiltinDeviceRegistry();
        const auto plan = m::LoadDevicePlan(argv[1], registry);
        Check(plan.driver == "agx_nero_revo2" && !plan.definition.effector.channels.size(),
              "recovery fixture requires the arm-only Nero adapter");
        const auto root = std::filesystem::canonical(plan.options.at("sdk_root"));
        std::ifstream marker(root / "TEST_FIXTURE_ONLY");
        std::string identity;
        std::getline(marker, identity);
        Check(identity == "manipulation-can-free-recovery-fixture", "refusing non-fixture SDK");
        Check(plan.options.at("channel").find("_recover") != std::string::npos,
              "refusing non-fixture channel");
        const auto control = root / "pyAgxArm" / "recovery.json";
        Control(control, "{\"mode\":\"normal\"}");
        m::DeviceSession session(plan.definition,
                                 registry.Create(plan.driver, plan.definition, plan.options));
        session.Connect();
        session.Diagnose();
        const m::JointPoint target{std::vector<double>(7, 0.1), 0.01};
        unsigned calls = 0;
        const auto inject = [&] {  // Call 1 precedes submission; inject on the first poll after it.
            if (++calls == 2) Control(control, "{\"mode\":\"" + scenario + "\"}");
            return false;
        };
        const auto started = m::Clock::now();
        const auto failed = session.MoveArm(target, inject);
        Check(calls >= 2, "fault was not injected after submission");
        Check(m::Clock::now() - started < std::chrono::seconds(5), "unbounded failure handling");
        Check(failed.status == m::Result::kStopUnknown,
              "unverified physical stop reported as known");
        Check(session.Latched(), "failure did not latch the assembly");
        Reject([&] { session.MoveArm(target); });
        Reject([&] { session.EnableArm(); });
        Reject([&] { session.Diagnose(); });

        // Removing the injected fault must not clear the latch or replay a command.
        Control(control,
                "{\"mode\":\"normal\",\"joints\":[0.125,-0.125,0.25,-0.25,0.375,-0.375,0.5]}");
        Reject([&] { session.MoveArm(target); });
        session.Disconnect();
        session.Connect();
        Check(!session.Latched(), "explicit reconnect did not clear the latch");
        const auto actual = session.ReadArm();
        const std::vector<double> expected{0.125, -0.125, 0.25, -0.25, 0.375, -0.375, 0.5};
        Check(actual.joints_rad == expected && !actual.moving && !actual.fault,
              "reconnect did not read independent actual state");
        std::cout << "scenario=" << scenario
                  << " result=stop_unknown latched=1 reconnect=idle"
                     " actual_state_verified=1\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
