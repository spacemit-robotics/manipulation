// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Local bounded JSONL binding; one serialized device writer, out-of-band cancellation.
#include "device_service.h"
#include <poll.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace m = spacemit::manipulation;

std::string Quote(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char ch : text) {
        if (ch == '"' || ch == '\\')
            out << '\\' << ch;
        else if (ch < 32)
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch) << std::dec;
        else
            out << ch;
    }
    return out.str() + '"';
}

std::string Values(const std::vector<double>& values) {
    std::ostringstream out;
    out << std::setprecision(17) << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ',';
        out << values[i];
    }
    return out.str() + ']';
}

// The controller's own joint limits, in the order and units the controller reports them.
std::string Limits(const std::vector<m::ControllerJointLimit>& limits) {
    std::ostringstream out;
    out << std::setprecision(17) << '[';
    for (std::size_t i = 0; i < limits.size(); ++i) {
        out << (i ? "," : "") << "{\"joint\":" << i + 1
            << ",\"min_angle_rad\":" << limits[i].minimum_rad
            << ",\"max_angle_rad\":" << limits[i].maximum_rad
            << ",\"max_velocity_rad_s\":" << limits[i].max_velocity_rad_s
            << ",\"max_acceleration_rad_s2\":" << limits[i].max_acceleration_rad_s2 << '}';
    }
    return out.str() + ']';
}

constexpr std::size_t kMaximumRequest = 1 << 20;  // Timed trajectories carry their knots.

void Keys(const YAML::Node& node, const std::set<std::string>& allowed) {
    if (!node.IsMap()) throw std::runtime_error("request must be an object");
    std::set<std::string> seen;
    for (const auto& pair : node) {
        const auto key = pair.first.as<std::string>();
        if (!allowed.count(key) || !seen.insert(key).second)
            throw std::runtime_error("unknown/duplicate request field");
    }
}

std::int64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(m::Clock::now().time_since_epoch())
            .count();
}
}  // namespace

std::string JsonString(const std::string& text) {
    return Quote(text);
}

std::string DeviceCapabilitiesJson(const m::DeviceSession& session) {
    const auto arm = session.GetArmCapabilities();
    std::ostringstream out;
    out << std::boolalpha << std::setprecision(17)
        << "{\"arm\":{\"joint_point\":" << arm.joint_point << ",\"joint_path\":" << arm.joint_path
        << ",\"joint_trajectory\":" << arm.joint_trajectory << ",\"enable\":" << arm.enable
        << ",\"disable\":" << arm.disable << ",\"verified_hold\":" << arm.verified_hold
        << ",\"minimum_speed_scale\":" << arm.minimum_speed_scale << "},\"effector\":";
    if (session.HasEffector()) {
        const auto tool = session.GetEffectorCapabilities();
        out << "{\"enable\":" << tool.enable << ",\"disable\":" << tool.disable
            << ",\"verified_hold\":" << tool.verified_hold
            << ",\"grasp_detection\":" << tool.grasp_detection
            << ",\"maximum_duration_s\":" << tool.maximum_duration_s << '}';
    } else {
        out << "null";
    }
    return out.str() + '}';
}

int ServeDevice(m::DeviceSession& session, bool allow_motion, bool allow_enable,
                const std::function<bool()>& interrupted, const std::string& driver,
                bool allow_disable, bool allow_effector_enable, bool allow_effector_disable) {
    std::mutex mutex, output_mutex;
    std::condition_variable changed;
    std::deque<YAML::Node> pending;
    std::atomic<bool> stopped{false};
    std::atomic<std::uint64_t> cancelled{0}, active{0};
    auto reply = [&](std::uint64_t id, bool ok, const std::string& data) {
        std::lock_guard<std::mutex> guard(output_mutex);
        std::cout << "{\"v\":1,\"id\":" << id << ",\"ok\":" << (ok ? "true" : "false")
                  << (ok ? ",\"data\":" : ",\"error\":") << data << "}\n"
                  << std::flush;
        if (!std::cout) {
            stopped = true;
            cancelled = active.load();
            changed.notify_one();
        }
    };
    std::cout << "{\"v\":1,\"event\":\"ready\",\"motion_allowed\":"
              << (allow_motion ? "true" : "false") << ",\"driver\":" << Quote(driver) << "}\n"
              << std::flush;
    if (!std::cout) return 2;
    std::thread reader([&] {
        std::string buffer;
        std::uint64_t last = 0;
        try {
            while (!stopped && !interrupted()) {
                struct pollfd item {
                    STDIN_FILENO, POLLIN, 0
                };
                const int status = poll(&item, 1, 100);
                if (status < 0) throw std::runtime_error("request stream failed");
                if (status == 0) continue;
                char bytes[2048];
                const auto size = read(STDIN_FILENO, bytes, sizeof(bytes));
                if (size <= 0) break;
                buffer.append(bytes, static_cast<std::size_t>(size));
                if (buffer.size() > kMaximumRequest)
                    throw std::runtime_error("request frame exceeds 1 MiB");
                std::size_t newline;
                while ((newline = buffer.find('\n')) != std::string::npos) {
                    const auto node = YAML::Load(buffer.substr(0, newline));
                    buffer.erase(0, newline + 1);
                    const auto id = node["id"].as<std::uint64_t>();
                    const auto op = node["op"].as<std::string>();
                    if (node["v"].as<int>() != 1 || id <= last)
                        throw std::runtime_error("invalid/replayed request identity");
                    last = id;
                    if (op == "quit") {
                        Keys(node, {"v", "id", "op", "deadline_ns"});
                        stopped = true;
                        break;
                    }
                    if (op == "cancel") {
                        Keys(node, {"v", "id", "op", "deadline_ns", "target_id"});
                        const auto target = node["target_id"].as<std::uint64_t>();
                        std::lock_guard<std::mutex> guard(mutex);
                        if (target == 0 ||
                            (target != active &&
                             (pending.empty() ||
                              target != pending.front()["id"].as<std::uint64_t>()))) {
                            reply(id, false, Quote("no matching active/pending command"));
                        } else {
                            cancelled = target;
                            reply(id, true, "{\"cancel_requested\":true}");
                        }
                        continue;
                    }
                    std::lock_guard<std::mutex> guard(mutex);
                    if (!pending.empty())
                        reply(id, false, Quote("bounded device queue busy"));
                    else {
                        pending.push_back(node);
                        changed.notify_one();
                    }
                }
            }
        } catch (const std::exception& error) {
            reply(0, false, Quote(error.what()));
        }
        stopped = true;
        cancelled = active.load();
        changed.notify_one();
    });
    bool deadline_latched = false;
    while (!stopped && !interrupted()) {
        YAML::Node request;
        {
            std::unique_lock<std::mutex> guard(mutex);
            changed.wait_for(guard, std::chrono::milliseconds(100),
                             [&] { return stopped || !pending.empty(); });
            if (stopped) break;
            if (pending.empty()) continue;
            request = pending.front();
            pending.pop_front();
            active = request["id"].as<std::uint64_t>();
        }
        const auto id = active.load();
        try {
            const auto op = request["op"].as<std::string>();
            auto allowed = std::set<std::string>{"v", "id", "op", "deadline_ns"};
            if (op == "arm_point") {
                allowed.insert({"target_rad", "speed_scale", "tolerance_rad", "wait_for_arrival",
                                "resend_interval_s"});
            }
            if (op == "arm_trajectory") {
                allowed.insert({"times_s", "positions_rad", "command_rate_hz", "max_step_rad",
                                "speed_scale", "tolerance_rad", "wait_for_arrival",
                                "effector_schedule"});
            }
            if (op == "arm_path") {
                allowed.insert({"path_rad", "speed_scale", "max_velocity_rad_s",
                                "max_acceleration_rad_s2", "tolerance_rad"});
            }
            if (op == "arm_follow") allowed.insert({"target_rad", "speed_scale"});
            if (op == "effector_position" || op == "effector_follow")
                allowed.insert({"positions", "duration_s"});
            if (op == "arm_jog") allowed.insert({"joint_index", "delta_rad", "speed_scale"});
            Keys(request, allowed);
            const auto deadline = request["deadline_ns"].as<std::int64_t>();
            if (Now() >= deadline) throw std::runtime_error("request expired before dispatch");
            const auto is_cancelled = [&] {
                return stopped || interrupted() || cancelled == id || Now() >= deadline;
            };
            std::string data;
            if (op == "capabilities") {
                data = DeviceCapabilitiesJson(session);
            } else if (op == "state" || op == "diagnose") {
                if (op == "diagnose") session.Diagnose();
                const auto arm = session.ReadArm();
                data = "{\"joints_rad\":" + Values(arm.joints_rad) +
                       ",\"enabled\":" + (arm.enabled ? "true" : "false") +
                       ",\"position_mode\":" + (arm.position_mode ? "true" : "false") +
                       ",\"moving\":" + (arm.moving ? "true" : "false") +
                       ",\"fault\":" + (arm.fault ? "true" : "false") +
                       ",\"firmware\":" + Quote(arm.firmware) +
                       ",\"arm_sample_id\":" + std::to_string(arm.sample_id) +
                       ",\"source_time_s\":" + std::to_string(arm.source_time_s);
                if (!arm.controller_limits.empty())
                    data += ",\"controller_limits\":" + Limits(arm.controller_limits);
                if (session.HasEffector()) {
                    const auto tool = session.ReadEffector();
                    const bool measured = session.Definition().effector.feedback ==
                                          m::EffectorFeedback::kPosition;
                    data += ",\"effector_feedback\":" + Quote(measured ? "position" : "none") +
                            ",\"effector_positions\":" + Values(tool.positions) +
                            ",\"effector_fault\":" + (tool.fault ? "true" : "false") +
                            ",\"effector_enabled\":" +
                            (tool.enabled.has_value() ? (*tool.enabled ? "true" : "false")
                                                      : "null") +
                            ",\"effector_loads\":" + Values(tool.loads) + ",\"effector_holding\":" +
                            (tool.holding.has_value() ? (*tool.holding ? "true" : "false")
                                                      : "null");
                }
                data += '}';
            } else {
                const bool holding = op == "arm_hold" || op == "effector_hold";
                if (!allow_motion || (deadline_latched && !holding)) {
                    throw std::runtime_error("motion forbidden or deadline latched");
                }
                if (op == "enable") {
                    if (!allow_enable)
                        throw std::runtime_error("enable requires explicit --enable permission");
                    session.EnableArm();
                    data = "{\"status\":\"succeeded\"}";
                } else if (op == "disable") {
                    if (!allow_disable)
                        throw std::runtime_error("disable requires --allow-disable permission");
                    session.DisableArm();
                    data = "{\"status\":\"succeeded\"}";
                } else if (op == "effector_enable") {
                    if (!allow_effector_enable) {
                        throw std::runtime_error(
                                "effector enable requires --allow-effector-enable permission");
                    }
                    session.EnableEffector();
                    data = "{\"status\":\"succeeded\"}";
                } else if (op == "effector_disable") {
                    if (!allow_effector_disable) {
                        throw std::runtime_error(
                                "effector disable requires --allow-effector-disable permission");
                    }
                    session.DisableEffector();
                    data = "{\"status\":\"succeeded\"}";
                } else {
                    m::CommandResult result;
                    if (op == "arm_hold") {
                        result = session.HoldArm();
                    } else if (op == "effector_hold") {
                        result = session.HoldEffector();
                    } else if (op == "arm_jog") {
                        result = session.JogArm(request["joint_index"].as<std::size_t>(),
                                                request["delta_rad"].as<double>(),
                                                request["speed_scale"].as<double>(), is_cancelled);
                    } else if (op == "arm_point") {
                        result = session.MoveArm(
                                {request["target_rad"].as<std::vector<double>>(),
                                 request["speed_scale"].as<double>(),
                                 request["tolerance_rad"] ? request["tolerance_rad"].as<double>()
                                                          : 0.0,
                                 !request["wait_for_arrival"] ||
                                         request["wait_for_arrival"].as<bool>(),
                                 request["resend_interval_s"]
                                         ? request["resend_interval_s"].as<double>()
                                         : 0.0},
                                is_cancelled);
                    } else if (op == "arm_follow") {
                        result = session.FollowArm({request["target_rad"].as<std::vector<double>>(),
                                                    request["speed_scale"].as<double>()});
                    } else if (op == "effector_follow") {
                        result = session.FollowEffector(
                                {request["positions"].as<std::vector<double>>(),
                                 request["duration_s"].as<double>()});
                    } else if (op == "arm_trajectory") {
                        std::vector<m::TimedEffectorPoint> schedule;
                        for (const auto& item : request["effector_schedule"]) {
                            Keys(item, {"at_s", "positions", "duration_s"});
                            schedule.push_back({item["at_s"].as<double>(),
                                                {item["positions"].as<std::vector<double>>(),
                                                 item["duration_s"].as<double>()}});
                        }
                        result = session.MoveArmTrajectory(
                                {request["times_s"].as<std::vector<double>>(),
                                 request["positions_rad"].as<std::vector<std::vector<double>>>(),
                                 request["command_rate_hz"].as<double>(),
                                 request["max_step_rad"].as<double>(),
                                 request["speed_scale"].as<double>(),
                                 request["tolerance_rad"] ? request["tolerance_rad"].as<double>()
                                                          : 0.0,
                                 !request["wait_for_arrival"] ||
                                         request["wait_for_arrival"].as<bool>()},
                                is_cancelled, schedule);
                    } else if (op == "arm_path") {
                        result = session.MoveArmPath(
                                {request["path_rad"].as<std::vector<std::vector<double>>>(),
                                 request["max_velocity_rad_s"].as<double>(),
                                 request["max_acceleration_rad_s2"].as<double>(),
                                 request["speed_scale"].as<double>(),
                                 request["tolerance_rad"] ? request["tolerance_rad"].as<double>()
                                                          : 0.0},
                                is_cancelled);
                    } else if (op == "effector_position") {
                        result = session.MoveEffector(
                                {request["positions"].as<std::vector<double>>(),
                                 request["duration_s"].as<double>()},
                                is_cancelled);
                    } else {
                        throw std::runtime_error("unsupported operation");
                    }
                    data = "{\"status\":" + Quote(m::ResultName(result.status)) +
                           ",\"detail\":" + Quote(result.detail) + ",\"physical_grip_verified\":" +
                           // Only measured grasp evidence (stopped on an object under load)
                           // verifies a grip.
                           (result.detail == "effector_grasp_holding" ? "true" : "false") + '}';
                }
            }
            if (Now() >= deadline) {
                deadline_latched = true;
                throw std::runtime_error("deadline exceeded; do not replay or infer physical stop");
            }
            reply(id, true, data);
        } catch (const std::exception& error) {
            reply(id, false, Quote(error.what()));
        }
        active = 0;
    }
    stopped = true;
    reader.join();
    return 0;
}
