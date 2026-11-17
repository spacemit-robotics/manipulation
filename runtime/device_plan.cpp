// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Strict v3 device composition. No model-specific configuration branches.
#include "manipulation/device_plan.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <set>
#include <stdexcept>

#include "manipulation/capability.h"

namespace spacemit::manipulation {
namespace {
void Keys(const YAML::Node& node, const std::set<std::string>& allowed) {
    if (!node.IsMap()) throw std::runtime_error("expected mapping");
    std::set<std::string> seen;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        if (!allowed.count(key) || !seen.insert(key).second)
            throw std::runtime_error("unknown/duplicate key: " + key);
    }
}
}  // namespace

DevicePlan LoadDevicePlan(const std::string& filename, const DeviceRegistry& registry) {
    return LoadDevicePlan(filename, registry, MakeBuiltinCapabilityRegistry());
}

DevicePlan LoadDevicePlan(const std::string& filename, const DeviceRegistry& registry,
        const CapabilityRegistry& capabilities) {
    if (std::filesystem::file_size(filename) > 65536)
        throw std::runtime_error("plan exceeds 64 KiB");
    const auto documents = YAML::LoadAllFromFile(filename);
    if (documents.size() != 1) throw std::runtime_error("one YAML document required");
    const auto root = documents[0];
    Keys(root, {"schema_version", "device", "steps"});
    if (root["schema_version"].as<int>() != 3)
        throw std::runtime_error("requires schema_version: 3");
    const auto device = root["device"];
    Keys(device, {"driver", "model", "arm", "effector", "options"});
    DevicePlan plan;
    if (device["driver"]) plan.driver = device["driver"].as<std::string>();
    if (device["model"]) {
        plan.model = device["model"].as<std::string>();
        if (plan.model.empty() || plan.model.size() > 80 ||
                plan.model.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-") !=
                        std::string::npos) {
            throw std::runtime_error("model must be a lowercase description identifier");
        }
    }
    auto& arm = plan.definition.arm;
    const auto node = device["arm"];
    Keys(node,
            {"device_id", "resource_id", "joints", "tolerance_rad", "timeout_s", "maximum_age_s",
                    "driver", "options"});
    arm.device_id = node["device_id"].as<std::string>();
    arm.resource_id = node["resource_id"].as<std::string>();
    if (node["tolerance_rad"]) arm.tolerance_rad = node["tolerance_rad"].as<double>();
    if (node["timeout_s"]) arm.timeout_s = node["timeout_s"].as<double>();
    if (node["maximum_age_s"]) arm.maximum_age_s = node["maximum_age_s"].as<double>();
    if (!node["joints"].IsSequence() || node["joints"].size() > 16)
        throw std::runtime_error("invalid joints");
    for (const auto& joint : node["joints"]) {
        Keys(joint, {"name", "minimum_rad", "maximum_rad"});
        arm.joints.push_back({joint["name"].as<std::string>(), joint["minimum_rad"].as<double>(),
                joint["maximum_rad"].as<double>()});
    }
    if (device["effector"]) {
        const auto effector = device["effector"];
        Keys(effector,
                {"device_id", "channels", "feedback", "tolerance", "timeout_s", "maximum_age_s",
                        "driver", "options", "resource_id"});
        auto& target = plan.definition.effector;
        target.device_id = effector["device_id"].as<std::string>();
        if (effector["resource_id"]) target.resource_id = effector["resource_id"].as<std::string>();
        target.channels = effector["channels"].as<std::vector<std::string>>();
        if (target.channels.empty())
            throw std::runtime_error("configured effector requires channels");
        if (effector["feedback"]) {
            const auto feedback = effector["feedback"].as<std::string>();
            if (feedback != "position" && feedback != "none")
                throw std::runtime_error("feedback must be position or none");
            target.feedback =
                    feedback == "none" ? EffectorFeedback::kNone : EffectorFeedback::kPosition;
        }
        if (effector["tolerance"]) target.tolerance = effector["tolerance"].as<double>();
        if (effector["timeout_s"]) target.timeout_s = effector["timeout_s"].as<double>();
        if (effector["maximum_age_s"])
            target.maximum_age_s = effector["maximum_age_s"].as<double>();
    }
    if (device["options"]) {
        if (!device["options"].IsMap() || device["options"].size() > 32)
            throw std::runtime_error("invalid options");
        for (const auto& option : device["options"]) {
            const auto key = option.first.as<std::string>();
            const auto value = option.second.as<std::string>();
            if (key.size() > 80 || value.size() > 4096 ||
                    !plan.options.emplace(key, value).second) {
                throw std::runtime_error("invalid/duplicate option");
            }
        }
    }
    const auto directory = std::filesystem::canonical(filename).parent_path().string();
    if (device["driver"]) {
        if (node["driver"] || node["options"] ||
                (device["effector"] &&
                        (device["effector"]["driver"] || device["effector"]["options"]))) {
            throw std::runtime_error(
                    "cannot mix device.driver with independently selected drivers");
        }
        if (!plan.definition.effector.resource_id.empty() &&
                plan.definition.effector.resource_id != plan.definition.arm.resource_id) {
            throw std::runtime_error("independent effector resource requires independent drivers");
        }
        const auto& descriptor = registry.Get(plan.driver);
        if (descriptor.resolve_options)
            plan.options = descriptor.resolve_options(plan.options, directory);
        registry.Validate(plan.driver, plan.definition, plan.options);
    } else {
        plan.composition = ParseDeviceComposition(device, &plan.definition, registry, directory);
    }
    const auto steps = root["steps"];
    if (!steps.IsSequence() || steps.size() > 128)
        throw std::runtime_error("steps must contain 0..128 entries");
    const auto context = DescribePlanCapabilities(plan, registry);
    for (const auto& step : steps) {
        plan.steps.push_back(capabilities.Parse(step, context));
    }
    return plan;
}

Devices CreatePlanDevices(const DevicePlan& plan, const DeviceRegistry& registry) {
    if (!plan.driver.empty()) return registry.Create(plan.driver, plan.definition, plan.options);
    return registry.CreateComposition(plan.definition, plan.composition);
}

CapabilityContext DescribePlanCapabilities(const DevicePlan& plan, const DeviceRegistry& registry) {
    auto context = DescribeDeviceCapabilities(plan.definition, CreatePlanDevices(plan, registry));
    if (!plan.driver.empty()) {
        // Preserve stricter legacy registration limits as well as driver-reported limits.
        const auto& descriptor = registry.Get(plan.driver);
        context.arm.minimum_speed_scale =
                std::max(context.arm.minimum_speed_scale, descriptor.minimum_arm_speed_scale);
        if (context.effector) {
            context.effector->maximum_duration_s = std::min(
                    context.effector->maximum_duration_s, descriptor.maximum_effector_duration_s);
        }
    }
    return context;
}

CommandResult RunDeviceSteps(DeviceSession& session, const std::vector<DeviceStep>& steps,
        const std::function<bool()>& cancelled) {
    return RunDeviceSteps(session, steps, MakeBuiltinCapabilityRegistry(), cancelled);
}

CommandResult RunDeviceSteps(DeviceSession& session, const std::vector<DeviceStep>& steps,
        const CapabilityRegistry& capabilities, const std::function<bool()>& cancelled) {
    CommandResult result;
    bool unverified = false;
    const auto context = DescribeSessionCapabilities(session);
    for (const auto& step : steps) capabilities.Validate(step, context);
    for (const auto& step : steps) {
        result = capabilities.Execute(session, step, cancelled);
        unverified = unverified || result.status == Result::kUnverified;
        if (result.status != Result::kSucceeded && result.status != Result::kUnverified)
            return result;
    }
    if (unverified) {  // The sequence finished, but at least one step was never measured.
        result.status = Result::kUnverified;
        result.detail = "sequence completed; one or more steps were not measured";
    }
    return result;
}
}  // namespace spacemit::manipulation
