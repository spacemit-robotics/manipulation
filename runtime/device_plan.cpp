// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Strict v3 device composition. No model-specific configuration branches.
#include "manipulation/device_plan.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <set>
#include <stdexcept>

namespace spacemit::manipulation {
namespace {
void Keys(const YAML::Node& node, const std::set<std::string>& allowed) {
    if (!node.IsMap()) throw std::runtime_error("expected mapping");
    std::set<std::string> seen;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        if (!allowed.count(key) || !seen.insert(key).second) throw std::runtime_error("unknown/duplicate key: " + key);
    }
}
}  // namespace
DevicePlan LoadDevicePlan(const std::string& filename, const DeviceRegistry& registry) {
    if (std::filesystem::file_size(filename) > 65536) throw std::runtime_error("plan exceeds 64 KiB");
    const auto documents = YAML::LoadAllFromFile(filename);
    if (documents.size() != 1) throw std::runtime_error("one YAML document required");
    const auto root = documents[0];
    Keys(root, {"schema_version", "device", "steps"});
    if (root["schema_version"].as<int>() != 3) throw std::runtime_error("requires schema_version: 3");
    const auto device = root["device"];
    Keys(device, {"driver", "model", "arm", "effector", "options"});
    DevicePlan plan;
    plan.driver = device["driver"].as<std::string>();
    if (device["model"]) {
        plan.model = device["model"].as<std::string>();
        if (plan.model.empty() || plan.model.size() > 80 ||
            plan.model.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos) {
            throw std::runtime_error("model must be a lowercase description identifier");
        }
    }
    auto& arm = plan.definition.arm;
    const auto node = device["arm"];
    Keys(node, {"device_id", "resource_id", "joints", "tolerance_rad", "timeout_s", "maximum_age_s"});
    arm.device_id = node["device_id"].as<std::string>();
    arm.resource_id = node["resource_id"].as<std::string>();
    if (node["tolerance_rad"]) arm.tolerance_rad = node["tolerance_rad"].as<double>();
    if (node["timeout_s"]) arm.timeout_s = node["timeout_s"].as<double>();
    if (node["maximum_age_s"]) arm.maximum_age_s = node["maximum_age_s"].as<double>();
    if (!node["joints"].IsSequence() || node["joints"].size() > 16) throw std::runtime_error("invalid joints");
    for (const auto& joint : node["joints"]) {
        Keys(joint, {"name", "minimum_rad", "maximum_rad"});
        arm.joints.push_back({joint["name"].as<std::string>(), joint["minimum_rad"].as<double>(),
            joint["maximum_rad"].as<double>()});
    }
    if (device["effector"]) {
        const auto effector = device["effector"];
        Keys(effector, {"device_id", "channels", "feedback", "tolerance", "timeout_s", "maximum_age_s"});
        auto& target = plan.definition.effector;
        target.device_id = effector["device_id"].as<std::string>();
        target.channels = effector["channels"].as<std::vector<std::string>>();
        if (target.channels.empty()) throw std::runtime_error("configured effector requires channels");
        if (effector["feedback"]) {
            const auto feedback = effector["feedback"].as<std::string>();
            if (feedback != "position" && feedback != "none") throw std::runtime_error("feedback must be position or none");
            target.feedback = feedback == "none" ? EffectorFeedback::kNone : EffectorFeedback::kPosition;
        }
        if (effector["tolerance"]) target.tolerance = effector["tolerance"].as<double>();
        if (effector["timeout_s"]) target.timeout_s = effector["timeout_s"].as<double>();
        if (effector["maximum_age_s"]) target.maximum_age_s = effector["maximum_age_s"].as<double>();
    }
    if (device["options"]) {
        if (!device["options"].IsMap() || device["options"].size() > 32) throw std::runtime_error("invalid options");
        for (const auto& option : device["options"]) {
            const auto key = option.first.as<std::string>(), value = option.second.as<std::string>();
            if (key.size() > 80 || value.size() > 4096 || !plan.options.emplace(key, value).second) {
                throw std::runtime_error("invalid/duplicate option");
            }
        }
    }
    const auto& descriptor = registry.Get(plan.driver);
    if (descriptor.resolve_options) {
        plan.options = descriptor.resolve_options(plan.options,
            std::filesystem::canonical(filename).parent_path().string());
    }
    registry.Validate(plan.driver, plan.definition, plan.options);
    const auto steps = root["steps"];
    if (!steps.IsSequence() || steps.size() > 128) throw std::runtime_error("steps must contain 0..128 entries");
    for (const auto& step : steps) {
        DeviceStep result;
        result.capability = step["capability"].as<std::string>();
        if (result.capability == "joint_point") {
            Keys(step, {"capability", "target_rad", "speed_scale"});
            result.arm.target_rad = step["target_rad"].as<std::vector<double>>();
            result.arm.speed_scale = step["speed_scale"].as<double>();
            if (result.arm.target_rad.size() != arm.joints.size() || !std::isfinite(result.arm.speed_scale) ||
                result.arm.speed_scale <= 0 || result.arm.speed_scale > 1 ||
                result.arm.speed_scale < registry.Get(plan.driver).minimum_arm_speed_scale) {
                throw std::runtime_error("invalid joint target or unsupported speed scale");
            }
            for (std::size_t i = 0; i < arm.joints.size(); ++i) {
                if (!std::isfinite(result.arm.target_rad[i]) || result.arm.target_rad[i] < arm.joints[i].minimum_rad ||
                    result.arm.target_rad[i] > arm.joints[i].maximum_rad) throw std::runtime_error("joint limit exceeded");
            }
        } else if (result.capability == "effector_position") {
            Keys(step, {"capability", "positions", "duration_s"});
            result.effector.positions = step["positions"].as<std::vector<double>>();
            result.effector.duration_s = step["duration_s"].as<double>();
            if (plan.definition.effector.channels.empty() ||
                result.effector.positions.size() != plan.definition.effector.channels.size() ||
                !std::isfinite(result.effector.duration_s) || result.effector.duration_s <= 0 ||
                result.effector.duration_s > registry.Get(plan.driver).maximum_effector_duration_s) {
                throw std::runtime_error("invalid effector target or unsupported duration");
            }
            for (double value : result.effector.positions) {
                if (!std::isfinite(value) || value < 0 || value > 1) throw std::runtime_error("effector limit exceeded");
            }
        } else { throw std::runtime_error("unknown capability"); }
        plan.steps.push_back(std::move(result));
    }
    return plan;
}
CommandResult RunDeviceSteps(DeviceSession& session, const std::vector<DeviceStep>& steps,
    const std::function<bool()>& cancelled) {
    CommandResult result;
    bool unverified = false;
    for (const auto& step : steps) {
        if (step.capability == "joint_point") result = session.MoveArm(step.arm, cancelled);
        else if (step.capability == "effector_position") result = session.MoveEffector(step.effector, cancelled);
        else throw std::runtime_error("unknown capability");
        unverified = unverified || result.status == Result::kUnverified;
        if (result.status != Result::kSucceeded && result.status != Result::kUnverified) return result;
    }
    if (unverified) {  // The sequence finished, but at least one step was never measured.
        result.status = Result::kUnverified;
        result.detail = "sequence completed; open-loop effector steps were not measured";
    }
    return result;
}
}  // namespace spacemit::manipulation
