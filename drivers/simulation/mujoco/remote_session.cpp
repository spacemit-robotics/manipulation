// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// No reconnect/replay after uncertainty. Local timestamps never substitute for remote freshness.
#include "remote_session.h"

#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace spacemit::manipulation::simulation {
namespace {
namespace w = wire;

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void Reject(bool condition, const char* message) {
    if (!condition) throw CommandRejected(message);
}

Clock::time_point Deadline(double seconds) {
    return Clock::now() +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

void Identity(const YAML::Node& reply, std::uint64_t id, const std::string& epoch) {
    w::Keys(reply, {"v", "id", "ok", "epoch", "data", "error", "rejected"});
    Require(w::Unsigned(reply["v"]) == 1 && w::Unsigned(reply["id"]) == id,
            "simulation response identity mismatch");
    Require(reply["epoch"].as<std::string>() == epoch, "simulation server epoch changed");
}
}  // namespace

RemoteSession::RemoteSession(Mapping mapping, std::optional<DeviceConfig> arm,
        std::optional<EffectorConfig> effector, TransportOptions transport) {
    mapping_ = std::move(mapping);
    arm_ = std::move(arm);
    effector_ = std::move(effector);
    transport_ = std::move(transport);
    if (arm_) ValidateArmMapping(*arm_, mapping_);
    if (effector_) ValidateEffectorMapping(*effector_, mapping_);
}

RemoteSession::~RemoteSession() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    Stop();
}

void RemoteSession::Acquire() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (owners_) {
        Ready();
        ++owners_;
        return;
    }
    stopped_ = false;
    failed_ = false;
    request_ = arm_required_ = effector_required_ = arm_source_ = effector_source_ = 0;
    arm_sample_ = {};
    effector_sample_ = {};
    try {
        const auto deadline = Deadline(2);
        socket_ = w::ConnectLoopback(transport_, deadline);
        w::WriteFrame(socket_,
                w::Object({{"v", "1"}, {"id", "1"}, {"op", w::Quote("hello")},
                        {"role", w::Quote("controller")},
                        {"contract", w::Quote(SessionContract(mapping_, arm_, effector_))}}),
                deadline);
        const auto reply = w::Parse(w::ReadFrame(socket_, deadline));
        epoch_ = reply["epoch"].as<std::string>();
        Require(epoch_.size() == 32, "invalid simulation server epoch");
        Identity(reply, 1, epoch_);
        Require(reply["ok"].as<bool>(), "simulation controller lease/contract rejected");
        const auto data = reply["data"];
        w::Keys(data, {"token", "fence", "lease_ms"});
        token_ = data["token"].as<std::string>();
        fence_ = w::Unsigned(data["fence"]);
        Require(token_.size() == 32 && fence_ > 0 && w::Unsigned(data["lease_ms"]) >= 500,
                "invalid simulation lease handshake");
        request_ = 1;
        heartbeat_socket_ = w::ConnectLoopback(transport_, deadline);
        w::WriteFrame(heartbeat_socket_,
                w::Object({{"v", "1"}, {"id", "1"}, {"op", w::Quote("hello")},
                        {"role", w::Quote("heartbeat")}, {"epoch", w::Quote(epoch_)},
                        {"token", w::Quote(token_)}, {"fence", std::to_string(fence_)}}),
                deadline);
        const auto heartbeat = w::Parse(w::ReadFrame(heartbeat_socket_, deadline));
        Identity(heartbeat, 1, epoch_);
        Require(heartbeat["ok"].as<bool>(), "simulation heartbeat rejected");
        owners_ = 1;
        heartbeat_ = std::thread(&RemoteSession::Heartbeat, this);
        Refresh();
    } catch (...) {
        Stop();
        throw;
    }
}

void RemoteSession::Release() noexcept {
    std::lock_guard<std::mutex> lifecycle(lifecycle_);
    bool last = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (owners_) last = --owners_ == 0;
    }
    if (last) Stop();
}

void RemoteSession::Stop() noexcept {
    stopped_ = true;
    if (heartbeat_socket_ >= 0) shutdown(heartbeat_socket_, SHUT_RDWR);
    if (socket_ >= 0) shutdown(socket_, SHUT_RDWR);
    if (heartbeat_.joinable()) heartbeat_.join();
    if (heartbeat_socket_ >= 0) close(heartbeat_socket_);
    if (socket_ >= 0) close(socket_);
    heartbeat_socket_ = socket_ = -1;
    owners_ = 0;
}

void RemoteSession::Ready() const {
    Require(owners_ && !stopped_ && !failed_ && socket_ >= 0,
            "remote simulation disconnected/latched; stop unknown");
}

YAML::Node RemoteSession::Call(const std::string& op, const std::string& fields, double timeout_s) {
    Ready();
    const auto id = ++request_;
    const auto deadline = Deadline(timeout_s);
    auto request = w::Object({{"v", "1"}, {"id", std::to_string(id)}, {"op", w::Quote(op)},
            {"epoch", w::Quote(epoch_)}, {"token", w::Quote(token_)},
            {"fence", std::to_string(fence_)}});
    request.pop_back();
    request += fields + '}';
    try {
        w::WriteFrame(socket_, request, deadline);
        const auto reply = w::Parse(w::ReadFrame(socket_, deadline));
        Identity(reply, id, epoch_);
        if (!reply["ok"].as<bool>()) {
            const auto detail = reply["error"].as<std::string>();
            if (reply["rejected"] && reply["rejected"].as<bool>()) throw CommandRejected(detail);
            throw std::runtime_error(detail);
        }
        const auto data = reply["data"];
        if (op != "state") {
            w::Keys(data, {"command_seq", "stopped"});
            Require(w::Unsigned(data["command_seq"]) == id,
                    "simulation response command watermark mismatch");
            if (op == "arm_hold" || op == "effector_hold")
                Require(data["stopped"].as<bool>(), "remote simulation hold unconfirmed");
        }
        return data;
    } catch (const CommandRejected&) {
        throw;
    } catch (...) {
        failed_ = true;
        shutdown(socket_, SHUT_RDWR);
        throw;
    }
}

void RemoteSession::Heartbeat() noexcept {
    std::uint64_t id = 1;
    try {
        while (!stopped_) {
            const auto deadline = Deadline(0.5);
            w::WriteFrame(heartbeat_socket_,
                    w::Object({{"v", "1"}, {"id", std::to_string(++id)}, {"op", w::Quote("ping")},
                            {"epoch", w::Quote(epoch_)}, {"token", w::Quote(token_)},
                            {"fence", std::to_string(fence_)}}),
                    deadline);
            const auto reply = w::Parse(w::ReadFrame(heartbeat_socket_, deadline));
            Identity(reply, id, epoch_);
            Require(reply["ok"].as<bool>(), "simulation heartbeat lost lease");
            for (unsigned tick = 0; tick < 10 && !stopped_; ++tick)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (...) {
        if (!stopped_) {
            failed_ = true;
            shutdown(socket_, SHUT_RDWR);
        }
    }
}

void RemoteSession::Refresh() {
    const auto began = Clock::now();
    try {
        const auto data = Call("state");
        const auto now = Clock::now();
        const double rtt = std::chrono::duration<double>(now - began).count();
        w::Keys(data, {"arm", "effector"});
        if (arm_) {
            const auto node = data["arm"];
            w::Keys(node,
                    {"sample_id", "age_s", "sim_time", "positions", "moving", "active",
                            "command_seq"});
            const auto source = w::Unsigned(node["sample_id"]);
            const auto applied = w::Unsigned(node["command_seq"]);
            const auto source_age = w::Finite(node["age_s"]);
            const auto age = source_age + rtt;
            Require(source >= arm_source_ && source_age >= 0 && age <= arm_->maximum_age_s,
                    "remote arm stale/regressed feedback");
            if (source > arm_source_ && applied >= arm_required_) {
                ArmState state;
                state.joints_rad = w::Numbers(node["positions"], arm_->joints.size());
                state.source_time_s = w::Finite(node["sim_time"]);
                state.moving = node["moving"].as<bool>();
                state.command_active = node["active"].as<bool>();
                state.received_at = now;
                state.sample_id = ++arm_sequence_;
                state.valid = state.enabled = state.position_mode = true;
                state.firmware = "mujoco-tcp";
                arm_sample_ = std::move(state);
                arm_source_ = source;
                arm_remote_age_ = age;
            }
            Require(arm_sample_.valid &&
                            arm_remote_age_ +
                                            std::chrono::duration<double>(
                                                    now - arm_sample_.received_at)
                                                    .count() <=
                                    arm_->maximum_age_s,
                    "remote arm has no fresh post-command sample");
        }
        if (effector_) {
            const auto node = data["effector"];
            w::Keys(node, {"sample_id", "age_s", "positions", "moving", "active", "command_seq"});
            const auto source = w::Unsigned(node["sample_id"]);
            const auto applied = w::Unsigned(node["command_seq"]);
            const auto source_age = w::Finite(node["age_s"]);
            const auto age = source_age + rtt;
            Require(source >= effector_source_ && source_age >= 0 &&
                            age <= effector_->maximum_age_s,
                    "remote effector stale/regressed feedback");
            if (source > effector_source_ && applied >= effector_required_) {
                EffectorState state;
                state.positions = w::Numbers(node["positions"], effector_->channels.size());
                for (const auto value : state.positions)
                    Require(value >= 0 && value <= 1,
                            "invalid normalized remote effector position");
                state.moving = node["moving"].as<bool>();
                state.command_active = node["active"].as<bool>();
                state.received_at = now;
                state.sample_id = ++effector_sequence_;
                state.valid = true;
                state.enabled = true;
                state.side = "simulation";
                effector_sample_ = std::move(state);
                effector_source_ = source;
                effector_remote_age_ = age;
            }
            Require(effector_sample_.valid &&
                            effector_remote_age_ +
                                            std::chrono::duration<double>(
                                                    now - effector_sample_.received_at)
                                                    .count() <=
                                    effector_->maximum_age_s,
                    "remote effector has no fresh post-command sample");
        }
    } catch (...) {
        failed_ = true;
        if (socket_ >= 0) shutdown(socket_, SHUT_RDWR);
        throw;
    }
}

ArmState RemoteSession::ReadArm() {
    std::lock_guard<std::mutex> lock(mutex_);
    Require(arm_.has_value(), "remote simulation has no arm");
    Refresh();
    return arm_sample_;
}

EffectorState RemoteSession::ReadEffector() {
    std::lock_guard<std::mutex> lock(mutex_);
    Require(effector_.has_value(), "remote simulation has no effector");
    Refresh();
    return effector_sample_;
}

void RemoteSession::SubmitArm(const JointPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    Ready();
    Reject(arm_ && point.target_rad.size() == arm_->joints.size() &&
                    std::isfinite(point.speed_scale) && point.speed_scale > 0 &&
                    point.speed_scale <= 1 && point.resend_interval_s == 0,
            "invalid remote MuJoCo point");
    for (std::size_t index = 0; index < point.target_rad.size(); ++index) {
        const auto value = point.target_rad[index];
        Reject(std::isfinite(value) && value >= arm_->joints[index].minimum_rad &&
                        value <= arm_->joints[index].maximum_rad,
                "remote target outside limits");
    }
    const auto reply = Call("arm_point",
            ",\"target\":" + w::Array(point.target_rad) +
                    ",\"speed_scale\":" + w::Number(point.speed_scale));
    arm_required_ = w::Unsigned(reply["command_seq"]);
    Require(arm_required_ == request_, "remote arm acceptance watermark mismatch");
}

void RemoteSession::SubmitEffector(const EffectorPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    Ready();
    Reject(effector_ && point.positions.size() == effector_->channels.size() &&
                    std::isfinite(point.duration_s) && point.duration_s > 0 &&
                    point.duration_s <= 120,
            "invalid remote MuJoCo effector point");
    for (const auto value : point.positions)
        Reject(std::isfinite(value) && value >= 0 && value <= 1, "invalid effector target");
    const auto reply = Call("effector_point",
            ",\"positions\":" + w::Array(point.positions) +
                    ",\"duration_s\":" + w::Number(point.duration_s));
    effector_required_ = w::Unsigned(reply["command_seq"]);
    Require(effector_required_ == request_, "remote effector acceptance watermark mismatch");
}

bool RemoteSession::Hold(bool arm) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto reply = Call(arm ? "arm_hold" : "effector_hold", "", mapping_.hold_timeout_s + 0.5);
    const auto sequence = w::Unsigned(reply["command_seq"]);
    Require(sequence == request_, "remote hold acceptance watermark mismatch");
    (arm ? arm_required_ : effector_required_) = sequence;
    return reply["stopped"].as<bool>();
}
}  // namespace spacemit::manipulation::simulation
