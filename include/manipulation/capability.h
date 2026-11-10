// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file capability.h
 * @brief Registered, configurable operations over the public device interfaces.
 */
#ifndef CAPABILITY_H
#define CAPABILITY_H

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/device.h"

namespace YAML {
class Node;
}

namespace spacemit::manipulation {
struct DeviceStep;

/// Offline device configuration and actual driver capabilities, without connecting hardware.
struct CapabilityContext {
    DeviceDefinition definition;
    ArmCapabilities arm;
    std::optional<EffectorCapabilities> effector;
};

/**
 * @brief One configurable capability; register before parsing or running tasks.
 *
 * Parsing plugins use yaml-cpp, but executions only depend on the public device interfaces.
 * parse and validate must not access hardware. execute must finish or cancel before returning,
 * retain the session's completion evidence, and never treat an unmeasured move as succeeded.
 * Registering and querying the same registry concurrently is not supported.
 */
struct CapabilityDescriptor {
    /// Stable lowercase configuration name.
    std::string id;
    /// Short description for capability discovery.
    std::string description;
    /// Decode the complete step mapping; reject unknown keys and invalid parameter types.
    std::function<DeviceStep(const YAML::Node&)> parse;
    /// Empty when supported; otherwise explain the missing device capability.
    std::function<std::string(const CapabilityContext&)> unsupported_reason;
    /// Validate parameters and configured bounds offline; throw before any device write.
    std::function<void(const DeviceStep&, const CapabilityContext&)> validate;
    /// Blocking execution; forward cancellation to every device operation.
    std::function<CommandResult(DeviceSession&, const DeviceStep&, const std::function<bool()>&)>
            execute;
};

/// Availability of one registered capability on a particular device configuration.
struct CapabilityAvailability {
    std::string id;
    std::string description;
    bool supported = false;
    std::string reason;
};

/// Configurable operations indexed by name; adding a capability needs no parser or runner branch.
class CapabilityRegistry {
public:
    /// Register a complete descriptor; duplicate or invalid names are rejected.
    void Register(CapabilityDescriptor descriptor);
    /// Find a capability, throwing when no descriptor has this name.
    const CapabilityDescriptor& Get(const std::string& id) const;
    /// Describe every registered operation without connecting to a device.
    std::vector<CapabilityAvailability> Describe(const CapabilityContext& context) const;
    /// Decode and validate one step offline; duplicate keys are always rejected.
    DeviceStep Parse(const YAML::Node& node, const CapabilityContext& context) const;
    /// Validate a parsed or programmatically constructed step offline.
    void Validate(const DeviceStep& step, const CapabilityContext& context) const;
    /// Validate against actual session capabilities, then run; pre-cancellation writes nothing.
    CommandResult Execute(DeviceSession& session, const DeviceStep& step,
            const std::function<bool()>& cancelled = {}) const;

private:
    std::map<std::string, CapabilityDescriptor> descriptors_;
};

/// Registry containing the capability modules selected at build time.
CapabilityRegistry MakeBuiltinCapabilityRegistry();

/// Snapshot capabilities from constructed drivers, without Connect(), leases or device I/O.
CapabilityContext DescribeDeviceCapabilities(
        const DeviceDefinition& definition, const Devices& devices);

/// Snapshot capabilities of an existing session; does not read live device state.
CapabilityContext DescribeSessionCapabilities(const DeviceSession& session);
}  // namespace spacemit::manipulation
#endif  // CAPABILITY_H
