/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_world.h
 * @brief Model-independent, explicitly stepped headless MuJoCo world.
 */
#ifndef MUJOCO_WORLD_H
#define MUJOCO_WORLD_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mujoco_sim {

class NativeSnapshot;

/** Scalar joint type; free and ball joints cannot be queried as scalars. */
enum class JointType { kHinge, kSlide };

/** Joint position/velocity use radians or metres and their per-second units. */
struct JointInfo {
    JointType type = JointType::kHinge;
    double position = 0.0;
    double velocity = 0.0;
    bool limited = false;
    double minimum = 0.0;
    double maximum = 0.0;
};

/** Control limits and affine position-servo metadata in model-defined units. */
struct ActuatorInfo {
    bool control_limited = false;
    double minimum = 0.0;
    double maximum = 0.0;
    // Stateless affine servo whose control is an unscaled position target.
    bool position_control = false;
    // Also accepts scaled targets, e.g. Robotiq control values from 0 to 255.
    bool affine_position_control = false;
    double gain = 0.0;
    double position_bias = 0.0;
    double velocity_bias = 0.0;
    // Direct joint transmission metadata; false for tendons and other targets.
    bool joint_transmission = false;
    std::string joint_name;
    // Nonzero scalar gear for a supported joint or fixed-joint tendon target.
    double gear = 0.0;
    // Unique scalar joints, in model transmission order. Empty if unsupported.
    std::vector<std::string> controlled_joints;
    // Spatial tendons and nonscalar/unnamed/zero-coupling targets are unsupported.
    bool transmission_supported = false;
    // Accounts for both the global actuation flag and disabled actuator groups.
    bool enabled = false;
    // Current model control, including any value established by a keyframe.
    double control = 0.0;
};

/** Immutable owning state; copies share storage and survive World destruction. */
class WorldSnapshot {
public:
    /** Empty snapshots are allowed for transport slots; inspection rejects them. */
    WorldSnapshot();
    ~WorldSnapshot();
    WorldSnapshot(const WorldSnapshot&);
    WorldSnapshot& operator=(const WorldSnapshot&);
    bool IsValid() const;
    std::uint64_t SampleId() const;
    double Time() const;
    JointInfo Joint(const std::string& name) const;
    /** True for a nonpositive-distance contact between the selected body trees. */
    bool TouchingBodies(const std::string& body1, const std::string& body2,
            bool include_descendants = true) const;

private:
    friend class World;
    friend class NativeSnapshot;
    class Impl;
    std::shared_ptr<const Impl> impl_;
};

/**
 * Owns one independent model/data pair, without rendering or worker threads.
 *
 * All calls require a single owner or external serialization. Construction
 * loads XML without changing cwd; model gravity, bodies and actuators are not
 * rewritten. No hardware access is performed. Invalid names/arguments throw
 * std::invalid_argument; model loading and invalid physics throw
 * std::runtime_error. After a physics failure, state/control operations fail
 * until an explicit successful Reset; no auto-reset state is published.
 */
class World {
public:
    /** Load XML and evaluate its default initial state. */
    explicit World(const std::string& xml_path);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    /** Reset to model defaults, or a named keyframe; invalid names change nothing. */
    void Reset(const std::string& keyframe = "");

    /** Read a named hinge/slide joint using its actual qpos/qvel addresses. */
    JointInfo Joint(const std::string& name) const;

    /** Read a named actuator, including tendon-driven actuators. */
    ActuatorInfo Actuator(const std::string& name) const;

    /**
     * Set a partial control batch. Reject unknown/duplicate names, nonfinite
     * values and out-of-range controls before writing any control; never clamp.
     * An empty batch is a no-op, and unmentioned actuators retain their controls.
     */
    void SetControls(const std::vector<std::pair<std::string, double>>& controls);

    /** Advance exactly count model timesteps; zero is a no-op. */
    void Step(std::size_t count = 1);

    /** Current simulation time in seconds; resets to the model/keyframe time. */
    double Time() const;

    /** Positive, finite model timestep in seconds. */
    double Timestep() const;

    /** Starts at one; increases after each successful step and explicit reset. */
    std::uint64_t SampleId() const;

    /** Copy state without advancing it; model storage is shared across snapshots. */
    WorldSnapshot Snapshot() const;

    /** Inspect current contacts without stepping; serialize with scene edits. */
    bool TouchingBodies(const std::string& body1, const std::string& body2,
            bool include_descendants = true) const;

    /**
     * Enable an existing body-based weld at the bodies' current relative pose.
     * Validation precedes all writes; does not move bodies or advance time.
     * The model weld must have a positive finite torque scale. This operation
     * imposes a constraint; it does not verify contact or prove a stable grasp.
     * Scene edits increment SampleId. Reset restores original XML weld settings.
     */
    void AttachWeld(
            const std::string& equality, const std::string& body1, const std::string& body2);

    /** Disable an existing body-based weld, without changing body poses. */
    void ReleaseWeld(const std::string& equality);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim

#endif  // MUJOCO_WORLD_H
