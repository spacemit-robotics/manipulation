// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Model-specific and configurable mapping roles with offline validation.
#include "session_backend.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <set>
#include <stdexcept>
#include <utility>

#include "manipulation/mujoco_driver.h"
#include "manipulation/resource_paths.h"
#include "remote_session.h"
#if MANIPULATION_HAS_MUJOCO_LOCAL
#include "mujoco_session.h"
#endif

namespace spacemit::manipulation::simulation {
namespace {
void Require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

void Keys(const YAML::Node& node, const std::set<std::string>& allowed) {
    Require(node.IsMap(), "MuJoCo mapping needs a mapping object");
    std::set<std::string> seen;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        Require(allowed.count(key) && seen.insert(key).second,
                "unknown/duplicate MuJoCo mapping key: " + key);
    }
}

std::string Name(const YAML::Node& node) {
    const auto value = node.as<std::string>();
    Require(!value.empty() && value.size() <= 200 && value.find('\0') == std::string::npos,
            "invalid MuJoCo model name");
    return value;
}

double Number(const YAML::Node& node) {
    const auto value = node.as<double>();
    Require(std::isfinite(value), "nonfinite MuJoCo mapping value");
    return value;
}

void Threshold(const YAML::Node& node, double* value, double maximum) {
    if (node) *value = Number(node);
    Require(*value > 0 && *value <= maximum, "invalid MuJoCo threshold/timeout");
}

DriverOptions Resolve(const DriverOptions& options, const std::string& directory) {
    return ResolveResourceOptions(options, directory, {"mapping_file"}, {});
}

class SimulationArm final : public Arm {
public:
    explicit SimulationArm(std::shared_ptr<SessionBackend> session)
        : session_(std::move(session)) {}

    ~SimulationArm() override { Disconnect(); }

    ArmCapabilities GetCapabilities() const override { return {true, false, true}; }

    void Connect() override {
        if (connected_) throw std::runtime_error("MuJoCo arm already connected");
        session_->Acquire();
        connected_ = true;
    }

    void Disconnect() noexcept override {
        if (connected_) session_->Release();
        connected_ = false;
    }

    ArmState Read() override {
        Ready();
        return session_->ReadArm();
    }

    void Submit(const JointPoint& point) override {
        Ready();
        session_->SubmitArm(point);
    }

    bool Hold() override {
        Ready();
        return session_->Hold(true);
    }

private:
    void Ready() const {
        if (!connected_) throw std::runtime_error("MuJoCo arm disconnected");
    }

    std::shared_ptr<SessionBackend> session_;
    bool connected_ = false;
};

class SimulationEffector final : public EndEffector {
public:
    explicit SimulationEffector(std::shared_ptr<SessionBackend> session)
        : session_(std::move(session)) {}

    ~SimulationEffector() override { Disconnect(); }

    EffectorCapabilities GetCapabilities() const override { return {true, 120}; }

    void Connect() override {
        if (connected_) throw std::runtime_error("MuJoCo effector already connected");
        session_->Acquire();
        connected_ = true;
    }

    void Disconnect() noexcept override {
        if (connected_) session_->Release();
        connected_ = false;
    }

    EffectorState Read() override {
        Ready();
        return session_->ReadEffector();
    }

    void Submit(const EffectorPoint& point) override {
        Ready();
        session_->SubmitEffector(point);
    }

    bool Hold() override {
        Ready();
        return session_->Hold(false);
    }

private:
    void Ready() const {
        if (!connected_) throw std::runtime_error("MuJoCo effector disconnected");
    }

    std::shared_ptr<SessionBackend> session_;
    bool connected_ = false;
};
}  // namespace

Devices WrapSession(std::shared_ptr<SessionBackend> session, bool arm, bool effector) {
    Devices devices;
    if (arm) devices.arm = std::make_unique<SimulationArm>(session);
    if (effector) devices.effector = std::make_unique<SimulationEffector>(session);
    return devices;
}

TransportOptions ParseTransport(const DriverOptions& options) {
    TransportOptions transport;
    for (const auto& [key, value] : options) {
        Require(key == "mapping_file" || key == "transport" || key == "host" || key == "port",
                "unknown MuJoCo option: " + key);
        Require(value.find('\0') == std::string::npos, "NUL in MuJoCo option");
    }
    Require(options.count("mapping_file"), "MuJoCo requires mapping_file");
    if (options.count("transport")) {
        const auto& value = options.at("transport");
        Require(value == "local" || value == "tcp", "MuJoCo transport must be local or tcp");
        transport.remote = value == "tcp";
    }
    Require(transport.remote || (!options.count("host") && !options.count("port")),
            "MuJoCo host/port require tcp transport");
    if (options.count("host")) transport.host = options.at("host");
    Require(transport.host == "127.0.0.1" || transport.host == "localhost",
            "MuJoCo TCP only supports loopback; use an SSH tunnel for remote hosts");
    transport.host = "127.0.0.1";
    if (options.count("port")) {
        const auto& port = options.at("port");
        Require(!port.empty() && port.size() <= 5 &&
                        port.find_first_not_of("0123456789") == std::string::npos,
                "invalid MuJoCo TCP port");
        transport.port = static_cast<unsigned>(std::stoul(port));
        Require(transport.port > 0 && transport.port <= 65535, "invalid MuJoCo TCP port");
    }
    return transport;
}

std::shared_ptr<SessionBackend> CreateSession(const DriverOptions& options,
        std::optional<DeviceConfig> arm, std::optional<EffectorConfig> effector) {
    const auto transport = ParseTransport(options);
    auto mapping = LoadMapping(options);
    if (transport.remote) {
        return std::make_shared<RemoteSession>(
                std::move(mapping), std::move(arm), std::move(effector), transport);
    }
#if MANIPULATION_HAS_MUJOCO_LOCAL
    return std::make_shared<SharedSession>(std::move(mapping), std::move(arm), std::move(effector));
#else
    throw std::runtime_error("local MuJoCo support not built; use transport: tcp");
#endif
}

Mapping LoadMapping(const DriverOptions& options) {
    ParseTransport(options);
    Require(options.at("mapping_file").find('\0') == std::string::npos,
            "invalid MuJoCo mapping_file path");
    const auto path = std::filesystem::path(options.at("mapping_file"));
    Require(path.is_absolute(), "MuJoCo mapping_file must be a resolved absolute path");
    Require(std::filesystem::file_size(path) <= 262144, "MuJoCo mapping exceeds 256 KiB");
    const auto documents = YAML::LoadAllFromFile(path.string());
    Require(documents.size() == 1, "MuJoCo mapping requires one document");
    const auto root = documents.front();
    Keys(root,
            {"schema_version", "scene", "initial_keyframe", "arm", "effector",
                    "arm_moving_threshold_rad_s", "effector_moving_threshold", "hold_timeout_s",
                    "synchronized_joint_point"});
    Require(root["schema_version"].as<int>() == 1, "MuJoCo mapping requires schema_version 1");
    Mapping mapping;
    mapping.filename = std::filesystem::canonical(path).string();
    mapping.scene = (std::filesystem::path(mapping.filename).parent_path() / Name(root["scene"]))
                            .lexically_normal()
                            .string();
    if (root["initial_keyframe"]) mapping.keyframe = Name(root["initial_keyframe"]);
    if (root["synchronized_joint_point"])
        mapping.synchronized_joint_point = root["synchronized_joint_point"].as<bool>();
    std::set<std::string> joints;
    std::set<std::string> actuators;
    const auto arm = root["arm"];
    Require(!arm || (arm.IsSequence() && arm.size() <= 16), "invalid MuJoCo arm mappings");
    for (const auto& node : arm) {
        Keys(node, {"joint", "actuator", "max_velocity_rad_s"});
        ArmMapping item{
                Name(node["joint"]), Name(node["actuator"]), Number(node["max_velocity_rad_s"])};
        Require(item.max_velocity_rad_s > 0 && item.max_velocity_rad_s <= 10,
                "MuJoCo nominal velocity must be in (0, 10] rad/s");
        Require(joints.insert(item.joint).second && actuators.insert(item.actuator).second,
                "duplicate MuJoCo joint/actuator mapping");
        mapping.arm.push_back(std::move(item));
    }
    const auto effector = root["effector"];
    Require(!effector || (effector.IsSequence() && effector.size() <= 32),
            "invalid MuJoCo effector mappings");
    std::set<std::string> channels;
    for (const auto& node : effector) {
        Keys(node,
                {"channel", "joint", "actuator", "joint_closed", "joint_open", "control_closed",
                        "control_open"});
        EffectorMapping item{Name(node["channel"]), Name(node["joint"]), Name(node["actuator"]),
                Number(node["joint_closed"]), Number(node["joint_open"]),
                Number(node["control_closed"]), Number(node["control_open"])};
        const double joint_span = item.joint_open - item.joint_closed;
        const double control_span = item.control_open - item.control_closed;
        Require(std::isfinite(joint_span) && std::isfinite(control_span) &&
                        std::abs(joint_span) > 1e-9 && std::abs(control_span) > 1e-9,
                "MuJoCo effector endpoints must have finite nonzero spans");
        Require(channels.insert(item.channel).second && joints.insert(item.joint).second &&
                        actuators.insert(item.actuator).second,
                "duplicate MuJoCo channel/joint/actuator mapping");
        mapping.effector.push_back(std::move(item));
    }
    Require(!mapping.arm.empty() || !mapping.effector.empty(), "empty MuJoCo mapping");
    Threshold(root["arm_moving_threshold_rad_s"], &mapping.arm_moving_threshold_rad_s, 0.1);
    Threshold(root["effector_moving_threshold"], &mapping.effector_moving_threshold, 0.1);
    Threshold(root["hold_timeout_s"], &mapping.hold_timeout_s, 10);
    return mapping;
}

void ValidateArmMapping(const DeviceConfig& config, const Mapping& mapping) {
    Require(config.joints.size() == mapping.arm.size(), "MuJoCo arm mapping count mismatch");
    for (std::size_t index = 0; index < config.joints.size(); ++index) {
        Require(config.joints[index].name == mapping.arm[index].joint,
                "MuJoCo arm mapping must follow configured joint order");
    }
}

void ValidateEffectorMapping(const EffectorConfig& config, const Mapping& mapping) {
    Require(config.feedback == EffectorFeedback::kPosition,
            "MuJoCo effector requires position feedback");
    Require(!config.channels.empty() && config.channels.size() == mapping.effector.size(),
            "MuJoCo effector mapping count mismatch");
    for (std::size_t index = 0; index < config.channels.size(); ++index) {
        Require(config.channels[index] == mapping.effector[index].channel,
                "MuJoCo effector mapping must follow configured channel order");
    }
}

std::string ModelPrefix(const std::string& name, const std::string& suffix) {
    Require(name.size() >= suffix.size() &&
                    name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0,
            "MuJoCo model mapping name must end with " + suffix);
    return name.substr(0, name.size() - suffix.size());
}

void ValidateUr5eMapping(const Mapping& mapping) {
    const std::array<std::string, 6> actuators{
            "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"};
    Require(mapping.arm.size() == actuators.size(), "mujoco_ur5e requires six arm joints");
    const auto prefix = ModelPrefix(mapping.arm.front().joint, "shoulder_pan_joint");
    for (std::size_t index = 0; index < actuators.size(); ++index) {
        Require(mapping.arm[index].joint == prefix + actuators[index] + "_joint" &&
                        mapping.arm[index].actuator == prefix + actuators[index],
                "mujoco_ur5e requires ordered UR5e joint/actuator names with one prefix");
    }
}

void ValidateRobotiqMapping(const Mapping& mapping) {
    Require(mapping.effector.size() == 1, "mujoco_robotiq_2f85 requires one gripper channel");
    const auto& effector = mapping.effector.front();
    const auto prefix = ModelPrefix(effector.joint, "gripper_right_driver_joint");
    Require(effector.channel == "gripper" &&
                    effector.actuator == prefix + "gripper_fingers_actuator" &&
                    std::abs(effector.joint_closed - 0.8) <= 1e-6 &&
                    std::abs(effector.joint_open) <= 1e-6 &&
                    std::abs(effector.control_closed - 255.0) <= 1e-6 &&
                    std::abs(effector.control_open) <= 1e-6,
            "mujoco_robotiq_2f85 requires the Robotiq 2F-85 mapping and endpoints");
}
}  // namespace spacemit::manipulation::simulation

namespace spacemit::manipulation {
namespace {
constexpr std::array<const char*, 2> kMujocoArmDrivers{
        "mujoco_ur5e", "mujoco_mapped_arm"};
constexpr std::array<const char*, 2> kMujocoEffectorDrivers{
        "mujoco_robotiq_2f85", "mujoco_mapped_effector"};
}  // namespace

bool IsMujocoArmDriver(const std::string& name) {
    return std::find(kMujocoArmDrivers.begin(), kMujocoArmDrivers.end(), name) !=
            kMujocoArmDrivers.end();
}

bool IsMujocoEffectorDriver(const std::string& name) {
    return std::find(kMujocoEffectorDrivers.begin(), kMujocoEffectorDrivers.end(), name) !=
            kMujocoEffectorDrivers.end();
}

void RegisterMujocoDevice(DeviceRegistry* registry) {
    namespace s = simulation;
    for (const auto* name : kMujocoArmDrivers) {
        registry->RegisterArm({name, 1,
                [model_specific = std::string(name) == "mujoco_ur5e"](
                        const DeviceConfig& config, const DriverOptions& options) {
                    const auto mapping = s::LoadMapping(options);
                    s::ValidateArmMapping(config, mapping);
                    if (model_specific) s::ValidateUr5eMapping(mapping);
                },
                [](const DeviceConfig& config, const DriverOptions& options) {
                    return s::WrapSession(s::CreateSession(options, config, std::nullopt), true,
                            false).arm;
                },
                s::Resolve});
    }
    for (const auto* name : kMujocoEffectorDrivers) {
        registry->RegisterEffector({name, 1,
                [model_specific = std::string(name) == "mujoco_robotiq_2f85"](
                        const EffectorConfig& config, const DriverOptions& options) {
                    const auto mapping = s::LoadMapping(options);
                    s::ValidateEffectorMapping(config, mapping);
                    if (model_specific) s::ValidateRobotiqMapping(mapping);
                },
                [](const EffectorConfig& config, const DriverOptions& options) {
                    return s::WrapSession(s::CreateSession(options, std::nullopt, config), false,
                            true).effector;
                },
                s::Resolve});
    }
    for (const auto* arm_name : kMujocoArmDrivers) {
        for (const auto* effector_name : kMujocoEffectorDrivers) {
            registry->RegisterShared({arm_name, effector_name,
                    [](const DeviceDefinition&, const DriverOptions& arm,
                            const DriverOptions& effector) {
                        if (s::LoadMapping(arm).filename != s::LoadMapping(effector).filename) {
                            throw std::runtime_error(
                                    "shared MuJoCo roles require the same mapping_file");
                        }
                        const auto first = s::ParseTransport(arm);
                        const auto second = s::ParseTransport(effector);
                        if (first.remote != second.remote || first.host != second.host ||
                                first.port != second.port)
                            throw std::runtime_error(
                                    "shared MuJoCo roles require the same transport");
                    },
                    [](const DeviceDefinition& config, const DriverOptions& arm,
                            const DriverOptions&) {
                        return s::WrapSession(
                                s::CreateSession(arm, config.arm, config.effector), true, true);
                    }});
        }
    }
}
}  // namespace spacemit::manipulation
