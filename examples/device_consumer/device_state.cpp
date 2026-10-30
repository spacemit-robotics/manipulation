// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Installed consumer: no internal headers, vendor imports or motion writes.
#include "manipulation/device_plan.h"
#include <iostream>
#include <stdexcept>
int main(int argc, char** argv) {
    namespace m = spacemit::manipulation;
    try {
        if (argc != 2) throw std::runtime_error("device-state PLAN");
        auto registry = m::MakeBuiltinDeviceRegistry();
        auto plan = m::LoadDevicePlan(argv[1], registry);
        m::DeviceSession session(plan.definition, registry.Create(plan.driver, plan.definition, plan.options));
        session.Connect();
        auto state = session.ReadArm();
        std::cout << "joints=" << state.joints_rad.size() << " moving=" << state.moving << '\n';
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
