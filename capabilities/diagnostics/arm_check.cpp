// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Supervised bounded arm check: relative 0.01 rad, return, cancel and reconnect.
#include "arm_check.h"

#include <iomanip>
#include <stdexcept>

namespace spacemit::manipulation {
namespace {
constexpr double kStep = 0.01;
constexpr double kSpeed = 0.01;
}  // namespace

int RunArmCheck(const DevicePlan& plan, const DeviceRegistry& registry, bool timeout_only,
                const std::function<bool()>& interrupted, std::ostream& out) {
    auto definition = plan.definition;
    definition.arm.tolerance_rad = 0.002;
    DeviceSession session(definition, registry.Create(plan.driver, definition, plan.options));
    session.Connect();
    session.Diagnose();
    const auto before = session.ReadArm();
    if (!before.enabled || !before.position_mode || before.moving || before.fault) {
        throw std::runtime_error(
                "check requires an enabled, stationary, normal arm in position mode");
    }
    for (std::size_t i = 0; i < before.joints_rad.size(); ++i) {
        if (before.joints_rad[i] < definition.arm.joints[i].minimum_rad ||
            before.joints_rad[i] > definition.arm.joints[i].maximum_rad) {
            throw std::runtime_error("actual pose outside declared limits; refusing any motion");
        }
    }
    auto target = before.joints_rad;
    target[0] += kStep;
    if (target[0] > definition.arm.joints[0].maximum_rad) target[0] = before.joints_rad[0] - kStep;
    const JointPoint away{target, kSpeed}, back{before.joints_rad, kSpeed};

    if (timeout_only) {
        session.Disconnect();  // Release the lease for a session with a 1 ms deadline.
        auto fast = definition;
        fast.arm.timeout_s = 0.001;
        {
            DeviceSession quick(fast, registry.Create(plan.driver, fast, plan.options));
            quick.Connect();
            quick.Diagnose();
            if (interrupted()) return 1;
            const auto result = quick.MoveArm(away, interrupted);
            out << "timeout=" << ResultName(result.status) << ' ' << result.detail << '\n'
                << std::flush;
            if (result.status != Result::kTimedOut) return 1;
            bool latched = false;
            try {
                quick.MoveArm(back);
            } catch (const std::exception&) {
                latched = true;
            }
            if (!latched) throw std::runtime_error("timeout did not latch");
        }
        session.Connect();
        session.Diagnose();
        const auto restored = session.MoveArm(back, interrupted);
        out << "post_timeout_return=" << ResultName(restored.status) << '\n';
        return restored.status == Result::kSucceeded ? 0 : 1;
    }

    auto cycle = session.MoveArm(away, interrupted);
    if (cycle.status == Result::kSucceeded) cycle = session.MoveArm(back, interrupted);
    out << "point_roundtrip=" << ResultName(cycle.status) << ' ' << cycle.detail << '\n'
        << std::flush;
    if (cycle.status != Result::kSucceeded || interrupted()) return 1;

    unsigned polls = 0;  // First call is the pre-submission check; cancel on the first poll.
    const auto cancel = session.MoveArm(away, [&] { return interrupted() || ++polls > 1; });
    out << "cancel=" << ResultName(cancel.status) << ' ' << cancel.detail << '\n' << std::flush;
    if (cancel.status != Result::kCancelled) return 1;

    const auto returned = session.MoveArm(back, interrupted);
    out << "return=" << ResultName(returned.status) << '\n';
    if (returned.status != Result::kSucceeded) return 1;

    session.Disconnect();
    session.Connect();
    const auto after = session.ReadArm();
    out << std::setprecision(12) << "reconnected_actual_rad=[";
    for (std::size_t i = 0; i < after.joints_rad.size(); ++i)
        out << (i ? "," : "") << after.joints_rad[i];
    out << "] moving=" << after.moving << " fault=" << after.fault << '\n';
    return after.moving || after.fault ? 1 : 0;
}

}  // namespace spacemit::manipulation
