// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Nero V120 implementation. Vendor details do not escape this driver target.
#include "manipulation/device.h"
#include "manipulation/resource_paths.h"

#include <cmath>
#include <filesystem>  // NOLINT(build/c++17): target explicitly requires C++17.
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "agx_session.h"

namespace spacemit::manipulation {
namespace {
void ValidateNero(const DeviceConfig& config, const DriverOptions& options) {
    if (config.joints.size() != 7) throw std::runtime_error("Nero requires seven joints");
    for (std::size_t i = 0; i < config.joints.size(); ++i) {
        if (config.joints[i].name != "joint" + std::to_string(i + 1)) {
            throw std::runtime_error("Nero joint order must be joint1..joint7");
        }
    }
    for (const auto& entry : options) {
        if (entry.first != "channel" && entry.first != "sdk_root" && entry.first != "worker" &&
            entry.first != "python") throw std::runtime_error("unknown Nero driver option: " + entry.first);
    }
    for (const auto* key : {"channel", "sdk_root", "worker", "python"}) {
        const auto found = options.find(key);
        if (found == options.end() || found->second.empty()) throw std::runtime_error(std::string("missing ") + key);
        if (std::string(key) != "channel" && std::string(key) != "python" &&
            !std::filesystem::path(found->second).is_absolute()) {
            throw std::runtime_error("worker and sdk_root must be resolved resource paths");
        }
    }
    const auto python = std::filesystem::path(options.at("python"));
    if (python.has_parent_path() && !python.is_absolute()) {
        throw std::runtime_error("python must be a PATH command or a resolved resource path");
    }
    const auto& channel = options.at("channel");
    if (channel.size() > 15 || channel.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-") !=
        std::string::npos) throw std::runtime_error("invalid CAN channel");
    // Different resource IDs must not defeat ownership of the same CAN channel.
    if (config.resource_id != "agx_" + channel) throw std::runtime_error("resource_id must be agx_<channel>");
}

class NeroArm final : public Arm {
public:
    explicit NeroArm(std::shared_ptr<agx::SharedSession> connection) : worker_(std::move(connection)) {}
    ArmCapabilities GetCapabilities() const override { return {true, true, true, 0.01, true, true}; }
    void Connect() override {
        worker_->Acquire();
        connected_ = true;
    }
    void Disconnect() noexcept override { if (connected_) worker_->Release(); connected_ = false; }
    ArmState Read() override {
        const auto node = worker_->Call("read");
        ArmState state;
        state.joints_rad = node["joints_rad"].as<std::vector<double>>();
        state.sample_id = node["sample_id"].as<std::uint64_t>();
        state.received_at = Clock::time_point(std::chrono::duration_cast<Clock::duration>(
            std::chrono::nanoseconds(node["received_ns"].as<std::int64_t>())));
        state.source_time_s = node["source_time_s"].as<double>();
        state.valid = node["valid"].as<bool>();
        state.enabled = node["enabled"].as<bool>();
        state.position_mode = node["position_mode"].as<bool>();
        state.moving = node["moving"].as<bool>();
        state.fault = node["fault"].as<bool>();
        state.firmware = node["firmware"].as<std::string>();
        state.command_active = node["command_active"] && node["command_active"].as<bool>();
        for (const auto& limit : node["controller_limits"]) {
            state.controller_limits.push_back({limit["min_angle_rad"].as<double>(), limit["max_angle_rad"].as<double>(),
                limit["max_velocity_rad_s"].as<double>(), limit["max_acceleration_rad_s2"].as<double>()});
        }
        return state;
    }
    ArmState Diagnose() override { worker_->Call("diagnose"); return Read(); }
    void Enable() override { worker_->Call("enable"); }
    void Submit(const JointPoint& point) override {
        std::ostringstream fields;
        fields << std::setprecision(17) << ",\"speed_scale\":" << point.speed_scale
                << ",\"tolerance_rad\":" << point.tolerance_rad
                << ",\"resend_interval_s\":" << point.resend_interval_s << ",\"target_rad\":[";
        for (std::size_t i = 0; i < point.target_rad.size(); ++i) {
            if (i) fields << ',';
            fields << point.target_rad[i];
        }
        fields << ']';
        worker_->Call("submit", fields.str());
    }
    void SubmitPath(const JointPath& path) override {
        // The worker streams the profile at 50 Hz from its own fresh feedback (see agx_worker.py).
        std::ostringstream fields;
        fields << std::setprecision(17) << ",\"speed_scale\":" << path.speed_scale
                << ",\"max_velocity_rad_s\":" << path.max_velocity_rad_s
                << ",\"max_acceleration_rad_s2\":" << path.max_acceleration_rad_s2
                << ",\"tolerance_rad\":" << path.tolerance_rad << ",\"path_rad\":[";
        for (std::size_t i = 0; i < path.waypoints_rad.size(); ++i) {
            fields << (i ? ",[" : "[");
            for (std::size_t j = 0; j < path.waypoints_rad[i].size(); ++j) {
                if (j) fields << ',';
                fields << path.waypoints_rad[i][j];
            }
            fields << ']';
        }
        fields << ']';
        worker_->Call("submit_path", fields.str());
    }
    void SubmitTrajectory(const JointTrajectory& trajectory) override {
        // The worker streams the knots itself (see agx_worker.py), so they are sent once, compactly.
        // Scalars at full precision: a limit sent rounded up would exceed itself (3 deg -> 0.0523598776).
        std::ostringstream fields;
        fields << std::setprecision(17) << ",\"speed_scale\":" << trajectory.speed_scale
                << ",\"command_rate_hz\":" << trajectory.command_rate_hz
                << ",\"max_step_rad\":" << trajectory.max_step_rad
                << ",\"tolerance_rad\":" << trajectory.tolerance_rad << ",\"times_s\":[";
        for (std::size_t i = 0; i < trajectory.times_s.size(); ++i) fields << (i ? "," : "") << trajectory.times_s[i];
        fields << std::setprecision(12) << "],\"positions_rad\":[";
        for (std::size_t i = 0; i < trajectory.positions_rad.size(); ++i) {
            fields << (i ? ",[" : "[");
            for (std::size_t j = 0; j < trajectory.positions_rad[i].size(); ++j) {
                fields << (j ? "," : "") << trajectory.positions_rad[i][j];
            }
            fields << ']';
        }
        fields << ']';
        worker_->Call("submit_trajectory", fields.str());
    }
    bool Hold() override { return worker_->Call("hold")["held"].as<bool>(); }

private:
    std::shared_ptr<agx::SharedSession> worker_;
    bool connected_ = false;
};
}  // namespace

void agx::ValidateArm(const DeviceConfig& config, const DriverOptions& options) { ValidateNero(config, options); }
DriverOptions agx::ResolveOptions(const DriverOptions& options, const std::string& directory) {
    return ResolveResourceOptions(options, directory, {"sdk_root", "worker"}, {"python"});
}
std::unique_ptr<Arm> agx::MakeArm(const DeviceConfig& config, const DriverOptions& options,
    std::shared_ptr<agx::SharedSession> connection) {
    ValidateNero(config, options);
    return std::make_unique<NeroArm>(std::move(connection));
}
}  // namespace spacemit::manipulation
