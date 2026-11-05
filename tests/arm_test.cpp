// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Arm execution contract: lease, freshness, completion evidence, cancel, timeout and faults.
#include "arm_session.h"

#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace m = spacemit::manipulation;

namespace {
void Check(bool value, const char* detail) {
    if (!value) throw std::runtime_error(detail);
}

void Reject(const std::function<void()>& operation, const char* detail) {
    bool rejected = false;
    try {
        operation();
    } catch (const std::exception&) {
        rejected = true;
    }
    Check(rejected, detail);
}

class TestArm final : public m::Arm {
public:
    m::ArmCapabilities GetCapabilities() const override { return {true, true, true}; }

    void Connect() override { ++connections; }

    void Disconnect() noexcept override {}

    m::ArmState Read() override {
        if (read_failure) throw std::runtime_error("injected read failure");
        m::ArmState state;
        state.joints_rad = {position};
        state.received_at = stale ? m::Clock::now() - std::chrono::seconds(5) : m::Clock::now();
        state.sample_id = freeze ? sample : ++sample;
        state.valid = true;
        state.enabled = enabled;
        state.position_mode = true;
        state.fault = fault;
        return state;
    }

    void Enable() override { enabled = true; }

    void Submit(const m::JointPoint& point) override {
        if (reject) throw m::CommandRejected("injected precondition refusal");
        ++writes;
        if (write_failure) throw std::runtime_error("injected write failure after partial send");
        if (!stall) position = point.target_rad.at(0);
    }

    bool Hold() override {
        ++holds;
        return !stop_failure;
    }

    int connections = 0, writes = 0, holds = 0;
    double position = 0;
    std::uint64_t sample = 1;
    bool stall = false, stale = false, freeze = false, enabled = true, fault = false;
    bool stop_failure = false, write_failure = false, read_failure = false, reject = false;
};

void Reconnect(m::ArmSession& session) {
    session.Disconnect();
    session.Connect();
}

void TestLockDirectory(const std::filesystem::path& root) {
    const auto created = root / "created";
    m::PrepareLockDirectory(created.string());

    struct stat info {};

    Check(stat(created.c_str(), &info) == 0 && (info.st_mode & 07777) == 01777,
          "shared directory not sticky 1777");
    m::PrepareLockDirectory(created.string());  // Existing valid directory is accepted.
    const auto open = root / "open";
    std::filesystem::create_directory(open);
    std::filesystem::permissions(open, std::filesystem::perms::all);
    Reject([&] { m::PrepareLockDirectory(open.string()); }, "non-sticky shared directory accepted");
    const auto link = root / "link";
    std::filesystem::create_directory_symlink(created, link);
    Reject([&] { m::PrepareLockDirectory(link.string()); }, "symlinked lock directory accepted");
    Reject([&] { m::PrepareLockDirectory("relative/locks"); }, "relative lock directory accepted");
}
}  // namespace

int main() {
    auto pattern =
            (std::filesystem::temp_directory_path() / "manipulation-arm-test-XXXXXX").string();
    Check(mkdtemp(pattern.data()) != nullptr, "cannot create test directory");
    const std::filesystem::path root(pattern);
    try {
        TestLockDirectory(root);
        const auto directory = (root / "locks").string();
        // timeout_s must exceed the 0.3 s arrival window; the stall below waits past it.
        m::DeviceConfig config{"test", "arm", {{"joint", -1, 1}}, 0.001, 0.5, 0.2};
        auto arm = std::make_unique<TestArm>();
        auto* raw = arm.get();
        m::ArmSession session(config, std::move(arm), directory);
        session.Connect();
        Check(raw->connections == 1 && raw->writes == 0, "connect wrote to the device");
        m::ArmSession duplicate(config, std::make_unique<TestArm>(), directory);
        Reject([&] { duplicate.Connect(); }, "second owner acquired the lease");

        // Limits, completion evidence, busy rejection and verified cancel.
        Reject([&] { session.Submit({{2}, 0.05}); }, "out-of-limit target accepted");
        Check(raw->writes == 0, "rejected target was written");
        auto result = session.Submit({{0.1}, 0.05});
        while (result.status == m::Result::kRunning) result = session.Poll();
        Check(result.status == m::Result::kSucceeded, "reachable target did not succeed");
        raw->freeze = true;
        session.Submit({{0.2}, 0.05});
        Check(session.Poll().status == m::Result::kRunning,
              "cached feedback counted as completion");
        Reject([&] { session.Submit({{0.3}, 0.05}); }, "busy session accepted a second target");
        Check(session.Cancel().status == m::Result::kCancelled,
              "verified hold not reported as cancelled");
        raw->freeze = false;

        // A refusal before any write leaves the session usable and issues no stop.
        raw->reject = true;
        const auto holds = raw->holds;
        bool refused = false;
        try {
            session.Submit({{0}, 0.05});
        } catch (const m::CommandRejected&) {
            refused = true;
        }
        Check(refused && raw->holds == holds, "refusal was latched or triggered a stop");
        Check(session.GetResult().status == m::Result::kCancelled,
              "refusal replaced the previous result");
        raw->reject = false;

        // Stale feedback rejects new motion; stalled motion times out and latches.
        raw->stale = true;
        Reject([&] { session.Submit({{0}, 0.05}); }, "stale feedback accepted");
        raw->stale = false;
        raw->stall = true;
        session.Submit({{0.8}, 0.05});
        std::this_thread::sleep_for(std::chrono::milliseconds(520));
        Check(session.Poll().status == m::Result::kTimedOut, "stall did not time out");
        Reject([&] { session.Submit({{0}, 0.05}); }, "timeout did not latch");
        Reconnect(session);
        Check(session.Read().joints_rad.at(0) == 0.2, "reconnect replayed or homed");
        raw->stall = false;

        // Unconfirmed stop, write failure and read failure all latch without replay.
        session.Submit({{0}, 0.05});
        raw->stop_failure = true;
        Check(session.Cancel().status == m::Result::kStopUnknown,
              "unconfirmed stop reported as known");
        Reject([&] { session.Submit({{0}, 0.05}); }, "stop_unknown did not latch");
        Reconnect(session);
        raw->write_failure = true;
        Check(session.Submit({{0}, 0.05}).status == m::Result::kStopUnknown,
              "write failure reported as stopped");
        Reject([&] { session.Submit({{0}, 0.05}); }, "write failure did not latch");
        raw->write_failure = false;
        raw->stop_failure = false;
        Reconnect(session);
        session.Submit({{0.4}, 0.05});
        raw->read_failure = true;
        Check(session.Poll().status == m::Result::kFault,
              "read failure during motion not reported as fault");
        raw->read_failure = false;
        Reject([&] { session.Submit({{0}, 0.05}); }, "read failure did not latch");

        // A hold without fresh feedback is attempted, but cannot be confirmed: it latches.
        Reconnect(session);
        raw->read_failure = raw->stop_failure = true;
        const auto attempts = raw->holds;
        Check(session.Hold().status == m::Result::kStopUnknown && raw->holds == attempts + 1,
              "hold without feedback reported as confirmed or not attempted");
        raw->read_failure = raw->stop_failure = false;
        Reject([&] { session.Submit({{0}, 0.05}); }, "hold without feedback did not latch");

        // Explicit enable only; controller fault during motion latches.
        Reconnect(session);
        raw->enabled = false;
        Reject([&] { session.Submit({{0}, 0.05}); }, "disabled arm accepted motion");
        session.Enable();
        session.Submit({{0}, 0.05});
        raw->fault = true;
        Check(session.Poll().status == m::Result::kFault, "controller fault not reported");
        std::filesystem::remove_all(root);  // Owned mkdtemp tree.
        std::cout << "arm lease/freshness/completion/cancel/timeout/fault contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
