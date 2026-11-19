// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "manipulation/kinematics.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace m = spacemit::manipulation;
namespace {

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Action>
void Reject(Action action) {
    bool rejected = false;
    try {
        action();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, "invalid input accepted");
}

void RegistryContract() {
    m::KinematicsRegistry registry;
    Reject([&] { registry.Create("missing", {}); });
    Reject([&] { registry.Register("", {}); });
    auto empty = [](const m::KinematicsConfig&) -> std::unique_ptr<m::Kinematics> { return {}; };
    registry.Register("test", empty);
    Reject([&] { registry.Register("test", empty); });
    Reject([&] { registry.Create("test", {}); });
}

void SolverContract(const std::string& backend, const std::string& fixture,
        const std::string& so101) {
    auto registry = m::MakeBuiltinKinematicsRegistry();
    m::KinematicsConfig config{fixture, "base", "tip", {"shoulder", "elbow"}};
    auto solver = registry.Create(backend, config);
    const auto neutral = solver->Forward({0, 0});
    Check(std::abs(neutral.x - 2) < 1e-10 && std::abs(neutral.y) < 1e-10 &&
                    std::abs(neutral.z) < 1e-10,
            "FK ignores configured base");
    const auto bent = solver->Forward({-0.7, 0.4});
    Check(std::abs(bent.x - std::cos(-0.7) - std::cos(-0.3)) < 1e-10 &&
                    std::abs(bent.y - std::sin(-0.7) - std::sin(-0.3)) < 1e-10,
            "FK ignores joint mapping");
    m::InverseOptions options;
    options.timeout_s = 2;
    options.epsilon = 1e-6;
    options.position_weight = 0.5;
    const auto solution = solver->Inverse(bent, {-0.5, 0.5}, options);
    Check(solution.has_value(), "weighted IK did not converge");
    const auto reached = solver->Forward(*solution);
    Check(std::hypot(reached.x - bent.x, reached.y - bent.y) < 3e-6 &&
                    std::abs(reached.qz - bent.qz) < 3e-6,
            "IK error is not in base frame");
    const auto outside = solver->Forward({3, 0});
    Check(std::isfinite(outside.x), "FK must permit measured positions outside model limits");
    Check(!solver->Inverse({9, 9, 9}, {0, 0}, options), "unreachable target solved");
    options.max_iterations = 1;
    Check(!solver->Inverse(bent, {0, 0}, options), "iteration bound ignored");
    Reject([&] { solver->Forward({0}); });
    Reject([&] { solver->Forward({0, std::numeric_limits<double>::quiet_NaN()}); });
    Reject([&] { solver->Inverse({2, 0, 0, 1e300, 0, 0, 1e300}, {0, 0}, options); });
    Reject([&] { solver->Inverse({2, 0, 0, 0, 0, 0, 0}, {0, 0}, options); });
    config.base_link = "sibling";
    Reject([&] { registry.Create(backend, config); });
    config.base_link = "base";
    options.timeout_s = 0;
    Reject([&] { solver->Inverse(bent, {0, 0}, options); });
    options.timeout_s = 1e-15;
    Check(!solver->Inverse(neutral, {0, 0}, options), "expired request returned a solution");
    config.joint_names = {"shoulder", "shoulder"};
    Reject([&] { registry.Create(backend, config); });
    config.joint_names = {"elbow", "shoulder"};
    Reject([&] { registry.Create(backend, config); });
    config.joint_names = {"shoulder", "elbow", "gripper"};
    Reject([&] { registry.Create(backend, config); });
    config = {so101, "base_link", "gripper_frame_link",
            {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}};
    solver = registry.Create(backend, config);
    const std::vector<double> joints{0.1, -0.3, 0.4, 0.2, -0.1};
    options = {};
    options.timeout_s = 2;
    const auto target = solver->Forward(joints);
    auto seed = joints;
    seed[1] += 0.05;
    Check(solver->Inverse(target, seed, options).has_value(), "SO101 position IK failed");
    Check(solver->JointNames().size() == 5 && solver->LowerLimits().size() == 5,
            "off-chain gripper included in arm model");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        RegistryContract();
        if (argc == 3) {
            // Run every built-in backend against the same contract. A backend absent from this
            // build is skipped rather than failed, so the suite works with either backend alone.
            const char* backends[] = {"eigen", "pinocchio"};
            const auto registry = m::MakeBuiltinKinematicsRegistry();
            int exercised = 0;
            for (const auto* backend : backends) {
                // Only "unknown kinematics backend" means the backend is absent from this build;
                // any other rejection comes from the empty config and means the backend exists.
                bool available = true;
                try {
                    registry.Create(backend, {});
                } catch (const std::invalid_argument& error) {
                    const std::string message = error.what();
                    available = message.find("unknown kinematics backend") == std::string::npos;
                } catch (const std::exception&) {
                    available = true;
                }
                if (!available) continue;
                SolverContract(backend, argv[1], argv[2]);
                ++exercised;
            }
            Check(exercised > 0, "no built-in kinematics backend is available");
        }
        std::cout << "kinematics contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
