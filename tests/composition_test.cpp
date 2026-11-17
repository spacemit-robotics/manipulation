// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Independent device assembly and all-or-nothing resource ownership, without hardware.
#include "manipulation/device.h"
#include "manipulation/device_composition.h"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <iostream>
#include <stdexcept>
#include <string>

namespace m = spacemit::manipulation;

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Action>
void Reject(Action action, const char* message) {
    bool rejected = false;
    try {
        action();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, message);
}

bool Locked(const std::string& directory, const std::string& resource) {
    const auto filename = std::filesystem::path(directory) / (resource + ".lock");
    const int descriptor = open(filename.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    Check(descriptor >= 0, "cannot inspect lock");
    const bool locked = flock(descriptor, LOCK_EX | LOCK_NB) != 0;
    close(descriptor);
    return locked;
}

struct Probe {
    unsigned arm_connects = 0;
    unsigned effector_connects = 0;
    unsigned disconnects = 0;
    bool fail_arm = false;
    bool fail_effector = false;
    bool fail_read = false;
    std::string directory;
    std::vector<std::string> resources;
};

class ProbeArm final : public m::Arm {
public:
    explicit ProbeArm(std::shared_ptr<Probe> probe) : probe_(std::move(probe)) {}

    m::ArmCapabilities GetCapabilities() const override { return {}; }

    void Connect() override {
        ++probe_->arm_connects;
        for (const auto& resource : probe_->resources) {
            Check(Locked(probe_->directory, resource), "driver connected before all leases");
        }
        if (probe_->fail_arm) throw std::runtime_error("injected arm failure");
    }

    void Disconnect() noexcept override { ++probe_->disconnects; }

    m::ArmState Read() override {
        m::ArmState state;
        state.joints_rad = {0};
        state.received_at = m::Clock::now();
        state.sample_id = ++sample_;
        state.valid = true;
        state.enabled = true;
        state.position_mode = true;
        return state;
    }

    void Submit(const m::JointPoint&) override {}

    bool Hold() override { return true; }

private:
    std::shared_ptr<Probe> probe_;
    std::uint64_t sample_ = 0;
};

class ProbeEffector final : public m::EndEffector {
public:
    explicit ProbeEffector(std::shared_ptr<Probe> probe) : probe_(std::move(probe)) {}

    void Connect() override {
        ++probe_->effector_connects;
        if (probe_->fail_effector) throw std::runtime_error("injected effector failure");
    }

    void Disconnect() noexcept override { ++probe_->disconnects; }

    m::EffectorState Read() override {
        if (probe_->fail_read) throw std::runtime_error("injected feedback failure");
        m::EffectorState state;
        state.positions = {1};
        state.received_at = m::Clock::now();
        state.sample_id = ++sample_;
        state.valid = true;
        return state;
    }

    void Submit(const m::EffectorPoint&) override {}

private:
    std::shared_ptr<Probe> probe_;
    std::uint64_t sample_ = 0;
};

m::Devices ProbeDevices(const std::shared_ptr<Probe>& probe) {
    m::Devices devices;
    devices.arm = std::make_unique<ProbeArm>(probe);
    devices.effector = std::make_unique<ProbeEffector>(probe);
    return devices;
}

void TestRegistry(const std::string& directory) {
    auto registry = m::MakeBuiltinDeviceRegistry();
    m::EffectorConfig unused_effector;
    unused_effector.resource_id = "../outside";
    Reject([&] { m::ValidateEffector(unused_effector); },
            "unused effector bypassed resource validation");
    m::DeviceDefinition definition;
    definition.arm = {"arm", "a_resource", {{"joint", -1, 1}}};
    definition.effector = {"tool", {"finger"}};
    definition.effector.resource_id = "z_resource";
    m::DeviceComposition composition{"mock", "mock", {}, {}};
    Check(!registry.ListArms().empty() && !registry.ListEffectors().empty(),
            "independent drivers not registered");
    m::DeviceSession session(
            definition, registry.CreateComposition(definition, composition), directory);
    session.Connect();
    Check(session.MoveArm({{0.1}, 0.05}).status == m::Result::kSucceeded, "independent arm failed");
    Check(session.MoveEffector({{0.1}, 0.01}).status == m::Result::kSucceeded,
            "independent effector failed");
    session.Disconnect();

    definition.effector.resource_id = definition.arm.resource_id;
    m::DeviceSession shared(
            definition, registry.CreateComposition(definition, composition), directory);
    shared.Connect();
    shared.Disconnect();
    definition.effector.resource_id.clear();
    Reject([&] { registry.CreateComposition(definition, composition); },
            "missing independent resource accepted");
    definition.effector.resource_id = "z_resource";
    composition.arm_driver = "missing";
    Reject([&] { registry.CreateComposition(definition, composition); }, "unknown arm accepted");
    composition.arm_driver = "mock";
    composition.effector_driver = "missing";
    Reject([&] { registry.CreateComposition(definition, composition); },
            "unknown effector accepted");
    composition.effector_driver = "mock";
    definition.effector.feedback = m::EffectorFeedback::kNone;
    m::DeviceSession open_loop(
            definition, registry.CreateComposition(definition, composition), directory);
    open_loop.Connect();
    Check(!open_loop.GetEffectorCapabilities().verified_hold, "invented open-loop stop support");
    Check(open_loop.MoveEffector({{0.5}, 0.01}).status == m::Result::kUnverified,
            "open-loop composition claimed measured arrival");
    open_loop.Disconnect();

    // Third-party drivers participate through registration only, without parser/core branches.
    auto probe = std::make_shared<Probe>();
    registry.RegisterArm({"vendor_arm", 1, [](const m::DeviceConfig&, const m::DriverOptions&) {},
            [probe](const m::DeviceConfig&, const m::DriverOptions&) {
                return std::make_unique<ProbeArm>(probe);
            }});
    registry.RegisterEffector(
            {"vendor_tool", 1, [](const m::EffectorConfig&, const m::DriverOptions&) {},
                    [probe](const m::EffectorConfig&, const m::DriverOptions&) {
                        return std::make_unique<ProbeEffector>(probe);
                    }});
    composition = {"vendor_arm", "vendor_tool", {}, {}};
    registry.CreateComposition(definition, composition);
    Check(probe->arm_connects == 0 && probe->effector_connects == 0,
            "offline factory connected a device");
    definition.effector.resource_id = definition.arm.resource_id;
    Reject([&] { registry.CreateComposition(definition, composition); },
            "unregistered shared connection accepted");
}

void TestOwnership(const std::string& directory) {
    m::DeviceDefinition definition;
    definition.arm = {"arm", "z_resource", {{"joint", -1, 1}}};
    definition.effector = {"tool", {"finger"}};
    definition.effector.resource_id = "a_resource";
    auto probe = std::make_shared<Probe>();
    probe->directory = directory;
    probe->resources = {"z_resource", "a_resource"};
    m::DeviceSession owner(definition, ProbeDevices(probe), directory);
    owner.Connect();
    auto competitor = definition;
    std::swap(competitor.arm.resource_id, competitor.effector.resource_id);
    auto other = std::make_shared<Probe>();
    m::DeviceSession second(competitor, ProbeDevices(other), directory);
    Reject([&] { second.Connect(); }, "reversed resource assignment bypassed leases");
    Check(other->arm_connects == 0 && other->effector_connects == 0 && other->disconnects == 0,
            "busy assembly reached hardware");
    owner.Disconnect();
    second.Connect();
    second.Disconnect();

    for (int failure = 0; failure < 3; ++failure) {
        probe->fail_arm = failure == 0;
        probe->fail_effector = failure == 1;
        probe->fail_read = failure == 2;
        Reject([&] { owner.Connect(); }, "injected connection failure was lost");
        Check(!Locked(directory, "a_resource") && !Locked(directory, "z_resource"),
                "connection failure leaked a lease");
    }
    probe->fail_arm = probe->fail_effector = probe->fail_read = false;
    owner.Connect();
    owner.Disconnect();

    // The first lexical lock is released if acquiring the second lock fails.
    const auto late = std::filesystem::path(directory) / "z_resource.lock";
    const int blocker = open(late.c_str(), O_RDWR | O_CLOEXEC);
    Check(blocker >= 0 && flock(blocker, LOCK_EX | LOCK_NB) == 0, "cannot block resource");
    const auto before = probe->arm_connects;
    Reject([&] { owner.Connect(); }, "partially owned assembly connected");
    Check(probe->arm_connects == before && !Locked(directory, "a_resource"),
            "partial acquisition connected or leaked first lock");
    close(blocker);
}
}  // namespace

int main() {
    auto pattern =
            (std::filesystem::temp_directory_path() / "composition-contract-XXXXXX").string();
    const char* directory = mkdtemp(pattern.data());
    if (!directory) return 1;
    try {
        TestRegistry(directory);
        TestOwnership(directory);
        std::filesystem::remove_all(directory);
        std::cout << "composition contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        std::filesystem::remove_all(directory);
        return 1;
    }
}
