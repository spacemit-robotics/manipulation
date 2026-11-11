// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// One persistent local world, one fenced writer, independent observers and heartbeat watchdog.
#include "server.h"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <deque>
#include <future>
#include <list>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "manipulation/device_plan.h"
#include "manipulation/mujoco_driver.h"
#include "manipulation/mujoco_runtime.h"
#include "session_backend.h"
#include "wire_protocol.h"

namespace spacemit::manipulation {
namespace {
namespace s = simulation;
namespace w = s::wire;

Clock::time_point Deadline(double seconds) {
    return Clock::now() +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Reject(bool condition, const char* message) {
    if (!condition) throw CommandRejected(message);
}

const DriverOptions& LocalOptions(const DevicePlan& plan) {
    Require(plan.driver.empty() && IsMujocoArmDriver(plan.composition.arm_driver),
            "simulation server accepts only independently selected mujoco roles");
    const auto& options = plan.composition.arm_options;
    Require(!s::ParseTransport(options).remote, "simulation server needs a local plan");
    if (!plan.definition.effector.channels.empty()) {
        Require(IsMujocoEffectorDriver(plan.composition.effector_driver) &&
                        plan.definition.arm.resource_id == plan.definition.effector.resource_id &&
                        !s::ParseTransport(plan.composition.effector_options).remote &&
                        s::LoadMapping(options).filename ==
                                s::LoadMapping(plan.composition.effector_options).filename,
                "simulation server requires one shared local mujoco world");
    }
    return options;
}

std::string Boolean(bool value) {
    return value ? "true" : "false";
}

struct CaptureJob {
    SimulationCaptureRequest request;
    std::promise<SimulationCaptureReply> result;
    Clock::time_point deadline;
    std::atomic<bool> abandoned{false};
};

struct Connection {
    explicit Connection(int descriptor) : socket(descriptor) {}

    ~Connection() { close(socket); }

    int socket;
    std::thread thread;
    std::atomic<bool> done{false};
};

class Server {
public:
    Server(const DevicePlan& plan, const SimulationServerOptions& options)
        : runtime_(plan.definition, LocalOptions(plan)) {
        options_ = options;
        definition_ = plan.definition;
        mapping_ = s::LoadMapping(LocalOptions(plan));
        epoch_ = w::Nonce();
        Require(options.port <= 65535 && std::isfinite(options.lease_timeout_s) &&
                        options.lease_timeout_s >= 0.5 && options.lease_timeout_s <= 5,
                "invalid simulation listener/lease options");
        contract_ = s::SessionContract(mapping_, definition_.arm,
                definition_.effector.channels.empty()
                        ? std::nullopt
                        : std::optional<EffectorConfig>(definition_.effector));
        devices_ = runtime_.CreateDevices();
    }

    ~Server() { Stop(); }

    int Run() {
        // Startup is explicit. No network request can choose an XML path or reset this world.
        runtime_.Start();
        devices_.arm->Connect();
        if (devices_.effector) devices_.effector->Connect();
        listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        Require(listener_ >= 0, "simulation listener creation failed");

        struct sockaddr_in address {};

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(static_cast<std::uint16_t>(options_.port));
        Require(bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 &&
                        listen(listener_, 16) == 0,
                "simulation loopback listener bind failed");
        socklen_t length = sizeof(address);
        Require(getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &length) == 0,
                "simulation listener address unavailable");
        watchdog_ = std::thread(&Server::Watchdog, this);
        if (options_.ready) options_.ready(ntohs(address.sin_port), epoch_);
        while (!stopped_ && !(options_.cancelled && options_.cancelled())) {
            Reap();
            Capture();
            if (options_.tick && !options_.tick(runtime_)) break;

            struct pollfd item {
                listener_, POLLIN, 0
            };

            const auto ready = poll(&item, 1, 10);
            if (ready < 0 && errno == EINTR) continue;
            Require(ready >= 0, "simulation listener poll failed");
            if (ready && (item.revents & POLLIN)) {
                const int descriptor =
                        accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
                if (descriptor < 0) continue;
                const int no_delay = 1;
                if (setsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay))) {
                    close(descriptor);
                    continue;
                }
                if (connections_.size() >= 16) {
                    close(descriptor);
                    continue;
                }
                auto connection = std::make_unique<Connection>(descriptor);
                auto* pointer = connection.get();
                connection->thread = std::thread([this, pointer] {
                    Serve(pointer->socket);
                    pointer->done = true;
                });
                connections_.push_back(std::move(connection));
            }
        }
        Stop();
        return 0;
    }

private:
    std::string Reply(std::uint64_t id, const std::string& data) const {
        return w::Object({{"v", "1"}, {"id", std::to_string(id)}, {"ok", "true"},
                {"epoch", w::Quote(epoch_)}, {"data", data}});
    }

    void Fail(int socket, std::uint64_t id, const std::string& message, bool rejected) noexcept {
        try {
            w::WriteFrame(socket,
                    w::Object({{"v", "1"}, {"id", std::to_string(id)}, {"ok", "false"},
                            {"epoch", w::Quote(epoch_)}, {"error", w::Quote(message)},
                            {"rejected", Boolean(rejected)}}),
                    Deadline(0.2));
        } catch (...) {
        }
    }

    void ValidateIdentity(const YAML::Node& request, std::uint64_t* previous) const {
        const auto id = w::Unsigned(request["id"]);
        Require(w::Unsigned(request["v"]) == 1 && id > *previous,
                "invalid/replayed simulation request identity");
        Require(request["epoch"].as<std::string>() == epoch_, "simulation server epoch mismatch");
        *previous = id;
    }

    void ValidateLease(const YAML::Node& request, bool renew) {
        std::lock_guard<std::mutex> lock(lease_mutex_);
        Require(!token_.empty() && !revoking_ && !fault_ && Clock::now() < expires_ &&
                        request["token"].as<std::string>() == token_ &&
                        w::Unsigned(request["fence"]) == fence_,
                "simulation controller lease lost");
        if (renew) expires_ = Deadline(options_.lease_timeout_s);
    }

    void Revoke(const std::string& token) noexcept {
        {
            std::lock_guard<std::mutex> lock(lease_mutex_);
            if (token.empty() || token_ != token || revoking_) return;
            revoking_ = true;
        }
        bool stopped = false;
        try {
            std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
            stopped = runtime_.Hold();
        } catch (...) {
        }
        {
            std::lock_guard<std::mutex> lock(lease_mutex_);
            // No new lease is granted while revocation is checking fresh physical feedback.
            fault_ = fault_ || !stopped;
            token_.clear();
            revoking_ = false;
        }
    }

    void Watchdog() noexcept {
        while (!stopped_) {
            std::string expired;
            {
                std::lock_guard<std::mutex> lock(lease_mutex_);
                if (!token_.empty() && Clock::now() >= expires_) expired = token_;
            }
            if (expired.empty()) {
                try {
                    std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
                    bool timed_out = false;
                    if (!arm_target_.empty()) {
                        const auto state = devices_.arm->Read();
                        bool reached = !state.command_active && !state.moving;
                        for (std::size_t i = 0; i < arm_target_.size(); ++i)
                            reached = reached &&
                                    std::abs(state.joints_rad[i] - arm_target_[i]) <=
                                            definition_.arm.tolerance_rad;
                        if (reached)
                            arm_target_.clear();
                        else
                            timed_out = Clock::now() >= arm_deadline_;
                    }
                    if (!tool_target_.empty()) {
                        const auto state = devices_.effector->Read();
                        bool reached = !state.command_active && !state.moving;
                        for (std::size_t i = 0; i < tool_target_.size(); ++i)
                            reached = reached &&
                                    std::abs(state.positions[i] - tool_target_[i]) <=
                                            definition_.effector.tolerance;
                        if (reached)
                            tool_target_.clear();
                        else
                            timed_out = timed_out || Clock::now() >= tool_deadline_;
                    }
                    if (timed_out) {
                        std::lock_guard<std::mutex> lock(lease_mutex_);
                        expired = token_;
                    }
                } catch (...) {
                    std::lock_guard<std::mutex> lock(lease_mutex_);
                    fault_ = true;
                    expired = token_;
                }
            }
            if (!expired.empty()) Revoke(expired);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::string State() {
        const auto arm = devices_.arm->Read();
        const double age = std::chrono::duration<double>(Clock::now() - arm.received_at).count();
        Require(arm.valid && !arm.fault && age >= 0 && age <= definition_.arm.maximum_age_s,
                "simulation server arm feedback is stale/faulted");
        const auto arm_data = w::Object({{"sample_id", std::to_string(arm.sample_id)},
                {"age_s", w::Number(age)}, {"sim_time", w::Number(arm.source_time_s)},
                {"positions", w::Array(arm.joints_rad)}, {"moving", Boolean(arm.moving)},
                {"active", Boolean(arm.command_active)},
                {"command_seq", std::to_string(arm.sample_id > arm_after_ ? arm_command_ : 0)}});
        std::string tool_data = "null";
        if (devices_.effector) {
            const auto tool = devices_.effector->Read();
            const double tool_age =
                    std::chrono::duration<double>(Clock::now() - tool.received_at).count();
            Require(tool.valid && !tool.fault && tool_age >= 0 &&
                            tool_age <= definition_.effector.maximum_age_s,
                    "simulation server effector feedback is stale/faulted");
            tool_data = w::Object({{"sample_id", std::to_string(tool.sample_id)},
                    {"age_s", w::Number(tool_age)}, {"positions", w::Array(tool.positions)},
                    {"moving", Boolean(tool.moving)}, {"active", Boolean(tool.command_active)},
                    {"command_seq",
                            std::to_string(tool.sample_id > tool_after_ ? tool_command_ : 0)}});
        }
        return w::Object({{"arm", arm_data}, {"effector", tool_data}});
    }

    std::string Dispatch(const YAML::Node& request) {
        std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
        ValidateLease(request, true);
        const auto op = request["op"].as<std::string>();
        const auto id = w::Unsigned(request["id"]);
        if (op == "state") {
            w::Keys(request, {"v", "id", "op", "epoch", "token", "fence"});
            return State();
        }
        bool arm = true;
        if (op == "arm_point") {
            w::Keys(request, {"v", "id", "op", "epoch", "token", "fence", "target", "speed_scale"});
            JointPoint point;
            point.target_rad = w::Numbers(request["target"], definition_.arm.joints.size());
            point.speed_scale = w::Finite(request["speed_scale"]);
            devices_.arm->Submit(point);
            arm_target_ = point.target_rad;
            arm_deadline_ = Deadline(definition_.arm.timeout_s);
        } else if (op == "effector_point") {
            w::Keys(request,
                    {"v", "id", "op", "epoch", "token", "fence", "positions", "duration_s"});
            Reject(devices_.effector != nullptr, "simulation server has no effector");
            EffectorPoint point;
            point.positions =
                    w::Numbers(request["positions"], definition_.effector.channels.size());
            point.duration_s = w::Finite(request["duration_s"]);
            Reject(point.duration_s <= definition_.effector.timeout_s,
                    "effector duration exceeds timeout");
            devices_.effector->Submit(point);
            tool_target_ = point.positions;
            tool_deadline_ = Deadline(definition_.effector.timeout_s);
            arm = false;
        } else if (op == "arm_hold" || op == "effector_hold") {
            w::Keys(request, {"v", "id", "op", "epoch", "token", "fence"});
            arm = op == "arm_hold";
            Reject(arm || devices_.effector != nullptr, "simulation server has no effector");
            const bool stopped = arm ? devices_.arm->Hold() : devices_.effector->Hold();
            Require(stopped, "simulation hold unconfirmed; world requires recovery");
            (arm ? arm_target_ : tool_target_).clear();
        } else {
            throw CommandRejected("unsupported simulation operation");
        }
        // Read after acceptance: only a later physics sample can prove this command was applied.
        if (arm) {
            arm_command_ = id;
            arm_after_ = devices_.arm->Read().sample_id;
        } else {
            tool_command_ = id;
            tool_after_ = devices_.effector->Read().sample_id;
        }
        if (op == "arm_hold" || op == "effector_hold")
            return w::Object({{"command_seq", std::to_string(id)}, {"stopped", "true"}});
        return w::Object({{"command_seq", std::to_string(id)}});
    }

    void Observer(int socket, std::uint64_t* previous) {
        while (!stopped_) {
            const auto request = w::Parse(w::ReadFrame(socket, Deadline(5)));
            ValidateIdentity(request, previous);
            const auto id = *previous;
            try {
                const auto op = request["op"].as<std::string>();
                if (op == "describe" || op == "state") {
                    w::Keys(request, {"v", "id", "op", "epoch"});
                    std::string data;
                    {
                        std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
                        data = op == "describe" ? w::Object({{"contract", w::Quote(contract_)}})
                                                : State();
                    }
                    w::WriteFrame(socket, Reply(id, data), Deadline(0.5));
                    continue;
                }
                w::Keys(request,
                        {"v", "id", "op", "epoch", "camera", "width", "height", "min_depth_m",
                                "max_depth_m"});
                Reject(request["op"].as<std::string>() == "capture", "observer is read-only");
                Reject(static_cast<bool>(options_.capture), "camera rendering is not enabled");
                auto job = std::make_shared<CaptureJob>();
                job->request.camera = request["camera"].as<std::string>();
                job->request.width = request["width"].as<int>();
                job->request.height = request["height"].as<int>();
                job->request.min_depth_m = w::Finite(request["min_depth_m"]);
                job->request.max_depth_m = w::Finite(request["max_depth_m"]);
                Reject(job->request.camera.size() <= 200 &&
                                job->request.camera.find('\0') == std::string::npos &&
                                job->request.width > 0 && job->request.width <= 4096 &&
                                job->request.height > 0 && job->request.height <= 4096 &&
                                static_cast<std::size_t>(job->request.width) *
                                                job->request.height <=
                                        4194304 &&
                                job->request.min_depth_m > 0 &&
                                job->request.max_depth_m > job->request.min_depth_m,
                        "invalid simulation camera request");
                job->deadline = Deadline(5);
                auto future = job->result.get_future();
                {
                    std::lock_guard<std::mutex> lock(capture_mutex_);
                    Reject(captures_.size() < 2, "simulation capture queue busy");
                    captures_.push_back(job);
                }
                if (future.wait_until(job->deadline) != std::future_status::ready) {
                    job->abandoned = true;
                    throw std::runtime_error("simulation capture timed out");
                }
                const auto result = future.get();
                w::Parse(result.metadata);
                w::WriteFrame(socket, Reply(id, result.metadata), Deadline(1));
                w::WriteFrame(socket, result.payload, Deadline(5), w::kImageLimit);
            } catch (const std::exception& error) {
                Fail(socket, id, error.what(), true);
            }
        }
    }

    void Capture() {
        std::shared_ptr<CaptureJob> job;
        {
            std::lock_guard<std::mutex> lock(capture_mutex_);
            if (captures_.empty()) return;
            job = captures_.front();
            captures_.pop_front();
        }
        if (job->abandoned || Clock::now() >= job->deadline) return;
        try {
            job->result.set_value(options_.capture(runtime_, job->request));
        } catch (...) {
            job->result.set_exception(std::current_exception());
        }
    }

    void Serve(int socket) noexcept {
        std::string controller;
        std::uint64_t previous = 0;
        try {
            const auto hello = w::Parse(w::ReadFrame(socket, Deadline(1)));
            Require(w::Unsigned(hello["v"]) == 1 && w::Unsigned(hello["id"]) == 1 &&
                            hello["op"].as<std::string>() == "hello",
                    "invalid simulation handshake");
            previous = 1;
            const auto role = hello["role"].as<std::string>();
            if (role == "observer") {
                w::Keys(hello, {"v", "id", "op", "role"});
                w::WriteFrame(socket, Reply(1, "{\"role\":\"observer\"}"), Deadline(0.5));
                Observer(socket, &previous);
                return;
            }
            if (role == "heartbeat") {
                w::Keys(hello, {"v", "id", "op", "role", "epoch", "token", "fence"});
                Require(hello["epoch"].as<std::string>() == epoch_, "heartbeat epoch mismatch");
                ValidateLease(hello, true);
                w::WriteFrame(socket, Reply(1, "{}"), Deadline(0.5));
                while (!stopped_) {
                    const auto request =
                            w::Parse(w::ReadFrame(socket, Deadline(options_.lease_timeout_s)));
                    ValidateIdentity(request, &previous);
                    w::Keys(request, {"v", "id", "op", "epoch", "token", "fence"});
                    Require(request["op"].as<std::string>() == "ping",
                            "heartbeat is not a command channel");
                    ValidateLease(request, true);
                    w::WriteFrame(socket, Reply(previous, "{}"), Deadline(0.5));
                }
                return;
            }
            w::Keys(hello, {"v", "id", "op", "role", "contract"});
            Reject(role == "controller" && hello["contract"].as<std::string>() == contract_,
                    "simulation role/model contract mismatch");
            std::string greeting;
            {
                std::lock_guard<std::mutex> dispatch(dispatch_mutex_);
                std::lock_guard<std::mutex> lock(lease_mutex_);
                Reject(token_.empty() && !revoking_ && !fault_,
                        "simulation controller busy/faulted");
                controller = token_ = w::Nonce();
                ++fence_;
                expires_ = Deadline(options_.lease_timeout_s);
                arm_command_ = tool_command_ = arm_after_ = tool_after_ = 0;
                arm_target_.clear();
                tool_target_.clear();
                greeting = Reply(1,
                        w::Object({{"token", w::Quote(token_)}, {"fence", std::to_string(fence_)},
                                {"lease_ms",
                                        std::to_string(static_cast<unsigned>(
                                                options_.lease_timeout_s * 1000))}}));
            }
            w::WriteFrame(socket, greeting, Deadline(0.5));
            while (!stopped_) {
                // Heartbeats use their own socket, so a bounded Hold never starves the lease.
                {
                    std::lock_guard<std::mutex> lock(lease_mutex_);
                    if (token_ != controller || revoking_) break;
                }

                struct pollfd item {
                    socket, POLLIN, 0
                };

                const auto available = poll(&item, 1, 100);
                if (available < 0 && errno == EINTR) continue;
                Require(available >= 0, "simulation controller polling failed");
                if (!available) continue;
                const auto request = w::Parse(w::ReadFrame(socket, Deadline(0.5)));
                ValidateIdentity(request, &previous);
                try {
                    const auto result = Dispatch(request);
                    w::WriteFrame(socket, Reply(previous, result), Deadline(0.5));
                } catch (const CommandRejected& error) {
                    Fail(socket, previous, error.what(), true);
                }
            }
        } catch (const CommandRejected& error) {
            Fail(socket, previous ? previous : 1, error.what(), true);
        } catch (const std::exception& error) {
            Fail(socket, previous ? previous : 1, error.what(), false);
        } catch (...) {
        }
        if (!controller.empty()) Revoke(controller);
        shutdown(socket, SHUT_RDWR);
    }

    void Reap() {
        for (auto it = connections_.begin(); it != connections_.end();) {
            if ((*it)->done) {
                (*it)->thread.join();
                it = connections_.erase(it);
            } else
                ++it;
        }
    }

    void Stop() noexcept {
        if (stopped_.exchange(true)) return;
        if (listener_ >= 0) close(listener_);
        listener_ = -1;
        for (const auto& connection : connections_) shutdown(connection->socket, SHUT_RDWR);
        {
            std::lock_guard<std::mutex> lock(capture_mutex_);
            for (const auto& job : captures_) {
                try {
                    job->result.set_exception(std::make_exception_ptr(
                            std::runtime_error("simulation server stopping")));
                } catch (...) {
                }
            }
            captures_.clear();
        }
        for (const auto& connection : connections_)
            if (connection->thread.joinable()) connection->thread.join();
        connections_.clear();
        if (watchdog_.joinable()) watchdog_.join();
        if (devices_.effector) devices_.effector->Disconnect();
        if (devices_.arm) devices_.arm->Disconnect();
        runtime_.Stop();
    }

    SimulationServerOptions options_;
    DeviceDefinition definition_;
    s::Mapping mapping_;
    MujocoRuntime runtime_;
    Devices devices_;
    std::string epoch_;
    std::string contract_;
    int listener_ = -1;
    std::atomic<bool> stopped_{false};
    std::thread watchdog_;
    std::list<std::unique_ptr<Connection>> connections_;
    std::mutex lease_mutex_;
    std::mutex dispatch_mutex_;
    std::mutex capture_mutex_;
    std::deque<std::shared_ptr<CaptureJob>> captures_;
    std::string token_;
    std::uint64_t fence_ = 0;
    Clock::time_point expires_{};
    bool revoking_ = false;
    bool fault_ = false;
    std::uint64_t arm_command_ = 0;
    std::uint64_t tool_command_ = 0;
    std::uint64_t arm_after_ = 0;
    std::uint64_t tool_after_ = 0;
    std::vector<double> arm_target_;
    std::vector<double> tool_target_;
    Clock::time_point arm_deadline_{};
    Clock::time_point tool_deadline_{};
};
}  // namespace

int RunSimulationServer(const DevicePlan& plan, const SimulationServerOptions& options) {
    Server server(plan, options);
    return server.Run();
}
}  // namespace spacemit::manipulation
