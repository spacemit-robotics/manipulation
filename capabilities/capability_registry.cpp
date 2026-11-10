// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Capability discovery, offline validation and execution through public device interfaces.
#include "manipulation/capability.h"

#include <yaml-cpp/yaml.h>
#include <set>
#include <stdexcept>
#include <utility>

#include "manipulation/device_plan.h"

namespace spacemit::manipulation {
void CapabilityRegistry::Register(CapabilityDescriptor descriptor) {
    if (descriptor.id.empty() || descriptor.id.size() > 80 ||
            descriptor.id.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") !=
                    std::string::npos ||
            descriptor.description.empty() || !descriptor.parse || !descriptor.unsupported_reason ||
            !descriptor.validate || !descriptor.execute) {
        throw std::runtime_error("incomplete or invalid capability registration");
    }
    const auto id = descriptor.id;
    if (!descriptors_.emplace(id, std::move(descriptor)).second)
        throw std::runtime_error("duplicate capability: " + id);
}

const CapabilityDescriptor& CapabilityRegistry::Get(const std::string& id) const {
    const auto found = descriptors_.find(id);
    if (found == descriptors_.end()) throw std::runtime_error("unknown capability: " + id);
    return found->second;
}

std::vector<CapabilityAvailability> CapabilityRegistry::Describe(
        const CapabilityContext& context) const {
    std::vector<CapabilityAvailability> available;
    for (const auto& entry : descriptors_) {
        const auto& descriptor = entry.second;
        auto reason = descriptor.unsupported_reason(context);
        available.push_back({descriptor.id, descriptor.description, reason.empty(), reason});
    }
    return available;
}

DeviceStep CapabilityRegistry::Parse(
        const YAML::Node& node, const CapabilityContext& context) const {
    if (!node.IsMap()) throw std::runtime_error("capability step must be a mapping");
    std::set<std::string> keys;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        if (!keys.insert(key).second)
            throw std::runtime_error("duplicate capability parameter: " + key);
    }
    const auto id = node["capability"].as<std::string>();
    auto step = Get(id).parse(node);
    if (step.capability != id)
        throw std::runtime_error("capability parser returned a different identifier");
    Validate(step, context);
    return step;
}

void CapabilityRegistry::Validate(const DeviceStep& step, const CapabilityContext& context) const {
    const auto& descriptor = Get(step.capability);
    const auto reason = descriptor.unsupported_reason(context);
    if (!reason.empty()) throw std::runtime_error(step.capability + " unsupported: " + reason);
    descriptor.validate(step, context);
}

CommandResult CapabilityRegistry::Execute(DeviceSession& session, const DeviceStep& step,
        const std::function<bool()>& cancelled) const {
    Validate(step, DescribeSessionCapabilities(session));
    if (cancelled && cancelled())
        return {0, Result::kCancelled, "cancelled before capability execution"};
    return Get(step.capability).execute(session, step, cancelled);
}

CapabilityContext DescribeDeviceCapabilities(
        const DeviceDefinition& definition, const Devices& devices) {
    if (!devices.arm ||
            static_cast<bool>(devices.effector) != !definition.effector.channels.empty())
        throw std::runtime_error("capability discovery requires a complete device assembly");
    CapabilityContext context{definition, devices.arm->GetCapabilities(), std::nullopt};
    if (devices.effector) context.effector = devices.effector->GetCapabilities();
    return context;
}

CapabilityContext DescribeSessionCapabilities(const DeviceSession& session) {
    CapabilityContext context{session.Definition(), session.GetArmCapabilities(), std::nullopt};
    if (session.HasEffector()) context.effector = session.GetEffectorCapabilities();
    return context;
}
}  // namespace spacemit::manipulation
