// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/** @file kinematics.h
 * @brief Offline, model-based forward and inverse kinematics; never opens a device.
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace spacemit::manipulation {

/// Tip pose expressed in the configured base frame, metres and a unit quaternion (w, x, y, z).
struct KinematicPose {
    double x = 0;
    double y = 0;
    double z = 0;
    double qw = 1;
    double qx = 0;
    double qy = 0;
    double qz = 0;
};

/// A chain in base-to-tip joint order, matching command/feedback independently of the driver.
struct KinematicsConfig {
    std::string urdf_path;
    std::string base_link;
    std::string tip_link;
    std::vector<std::string> joint_names;
};

/// Bounded damped least-squares search. A result is not a collision-free motion plan.
struct InverseOptions {
    double timeout_s = 0.1;
    double epsilon = 1e-4;
    /// Weight in [0,1]; 1 solves position only, 0 solves orientation only.
    double position_weight = 1.0;
    int max_iterations = 1000;
};

/// One caller at a time. Bad inputs throw; a valid but unsolved IK request returns nullopt.
class Kinematics {
public:
    virtual ~Kinematics() = default;
    /// Compute measured poses even outside model limits; only dimensions and finiteness checked.
    virtual KinematicPose Forward(const std::vector<double>& joints_rad) = 0;
    /// Return a solution inside URDF limits, in JointNames() order, or nullopt on no convergence.
    virtual std::optional<std::vector<double>> Inverse(const KinematicPose& target,
            const std::vector<double>& seed_rad, const InverseOptions& options = {}) = 0;
    virtual const std::vector<std::string>& JointNames() const = 0;
    virtual const std::vector<double>& LowerLimits() const = 0;
    virtual const std::vector<double>& UpperLimits() const = 0;
};

/// Explicit registration; registering a backend does not open devices or load models.
class KinematicsRegistry {
public:
    using Factory = std::function<std::unique_ptr<Kinematics>(const KinematicsConfig&)>;
    void Register(const std::string& id, Factory factory);
    std::unique_ptr<Kinematics> Create(const std::string& id,
            const KinematicsConfig& config) const;

private:
    std::map<std::string, Factory> factories_;
};

/// Backends enabled in this build. Unavailable IDs fail explicitly, without a fallback.
KinematicsRegistry MakeBuiltinKinematicsRegistry();

}  // namespace spacemit::manipulation
#endif  // KINEMATICS_H
