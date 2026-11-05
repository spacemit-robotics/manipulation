// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Explicit register transactions with the centered-angle mapping and no writes on connection.
#include "so101_session.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <set>
#include <sstream>
#include <system_error>

#include "feetech_bus.h"

namespace spacemit::manipulation::so101 {
namespace {
constexpr double kPi = 3.14159265358979323846;
const std::array<std::string, kArmJoints> kJointNames{"shoulder_pan", "shoulder_lift", "elbow_flex",
                                                      "wrist_flex", "wrist_roll"};

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error("SO101: " + message);
}

std::string Value(const DriverOptions& options, const std::string& key, std::string fallback = {}) {
    const auto found = options.find(key);
    return found == options.end() ? fallback : found->second;
}

double Number(const std::string& text) {
    size_t length = 0;
    const double result = std::stod(text, &length);
    Require(length == text.size() && std::isfinite(result), "invalid number: " + text);
    return result;
}

unsigned Unsigned(const std::string& text, unsigned maximum) {
    const double value = Number(text);
    Require(value >= 1 && value <= maximum && std::floor(value) == value,
            "invalid integer: " + text);
    return static_cast<unsigned>(value);
}

int Integer(const YAML::Node& node, const char* key, int minimum, int maximum) {
    Require(node[key] && node[key].IsScalar(), std::string("missing calibration field: ") + key);
    const double value = Number(node[key].as<std::string>());
    Require(value >= minimum && value <= maximum && std::floor(value) == value,
            std::string("invalid calibration field: ") + key);
    return static_cast<int>(value);
}

YAML::Node CalibrationFile(const std::string& path) {
    Require(std::filesystem::is_regular_file(path),
            "calibration file not found: " + path + "; create it with `so101-calibrate capture`");
    Require(std::filesystem::file_size(path) <= 65536, "calibration file is too large");
    auto root = YAML::LoadFile(path);
    Require(root.IsMap(), "calibration must be an object");
    std::set<std::string> keys;
    for (const auto& field : root) {
        Require(keys.insert(field.first.as<std::string>()).second, "duplicate calibration field");
        if (field.second.IsMap()) {
            std::set<std::string> nested;
            for (const auto& value : field.second) {
                Require(nested.insert(value.first.as<std::string>()).second,
                        "duplicate joint field");
            }
        }
    }
    return root;
}

int Word(const std::vector<uint8_t>& data, size_t offset) {
    return data.at(offset) | (static_cast<int>(data.at(offset + 1)) << 8);
}
}  // namespace

double JointPosition(int ticks) {
    return ticks * (2 * kPi / 4095.0) - kPi;
}

int JointTicks(double radians) {
    if (!std::isfinite(radians) || radians < -kPi || radians >= kPi) {
        throw CommandRejected("SO101 joint target is outside the encoder range");
    }
    return static_cast<int>(std::lround((radians + kPi) * 4095.0 / (2 * kPi)));
}

Settings Parse(const DeviceDefinition& definition, const DriverOptions& options) {
    const std::set<std::string> keys{"port",
                                     "arm_ids",
                                     "gripper_id",
                                     "arm_calibration",
                                     "gripper_calibration",
                                     "max_velocity_rad_s",
                                     "grip_load_threshold"};
    for (const auto& entry : options)
        Require(keys.count(entry.first), "unknown option: " + entry.first);
    Settings settings;
    settings.port = Value(options, "port");
    Require(!settings.port.empty(), "port is required");
    settings.gripper_id = Unsigned(Value(options, "gripper_id", "6"), 253);
    settings.maximum_velocity = Number(Value(options, "max_velocity_rad_s", "1.0"));
    settings.grip_load_threshold =
            static_cast<int>(Unsigned(Value(options, "grip_load_threshold", "100"), 1000));
    Require(settings.maximum_velocity >= 0.01 && settings.maximum_velocity <= 3.6,
            "max_velocity_rad_s must be 0.01..3.6");
    std::istringstream ids(Value(options, "arm_ids", "1,2,3,4,5"));
    std::string entry;
    size_t index = 0;
    std::set<uint8_t> seen;
    while (std::getline(ids, entry, ',')) {
        Require(index < kArmJoints, "arm_ids must contain five IDs");
        settings.ids[index] = Unsigned(entry, 253);
        Require(seen.insert(settings.ids[index]).second, "duplicate arm ID");
        ++index;
    }
    Require(index == kArmJoints && !Value(options, "arm_ids", "1,2,3,4,5").empty() &&
                    Value(options, "arm_ids", "1,2,3,4,5").back() != ',',
            "arm_ids must contain five IDs");
    Require(!seen.count(settings.gripper_id), "arm and gripper IDs overlap");
    Require(definition.arm.joints.size() == kArmJoints, "requires five arm joints");
    for (size_t i = 0; i < kArmJoints; ++i) {
        const auto& joint = definition.arm.joints[i];
        Require(joint.name == kJointNames[i], "joint names/order do not match SO101");
        Require(joint.minimum_rad >= -kPi && joint.maximum_rad < kPi,
                "joint limits must fit the non-wrapping encoder range");
    }
    const std::string arm_path = Value(options, "arm_calibration");
    if (!arm_path.empty()) {
        const auto calibration = CalibrationFile(arm_path);
        Require(calibration.size() == kArmJoints, "arm calibration must contain five named joints");
        for (size_t i = 0; i < kArmJoints; ++i) {
            const auto joint = calibration[kJointNames[i]];
            Require(joint && joint.IsMap(), "missing joint calibration: " + kJointNames[i]);
            Require(Integer(joint, "id", 1, 253) == settings.ids[i],
                    "calibration motor ID mismatch");
            auto& output = settings.calibration[i];
            output.offset = Integer(joint, "homing_offset", -2047, 2047);
            output.minimum = Integer(joint, "range_min", 0, 4095);
            output.maximum = Integer(joint, "range_max", 0, 4095);
            Require(output.minimum < output.maximum, "invalid calibrated arm range");
        }
        settings.arm_calibrated = true;
    }
    const bool gripper = !definition.effector.channels.empty();
    if (gripper) {
        Require(definition.effector.channels == std::vector<std::string>{"gripper"},
                "requires one channel named gripper");
        Require(definition.effector.feedback == EffectorFeedback::kPosition,
                "SO101 gripper requires position feedback");
        const auto path = Value(options, "gripper_calibration");
        Require(!path.empty(), "gripper_calibration is required for the gripper variant");
        const auto calibration = CalibrationFile(path);
        Require(Integer(calibration, "motor_id", 1, 253) == settings.gripper_id,
                "gripper calibration motor ID mismatch");
        settings.open_ticks = Number(calibration["open_ticks"].as<std::string>());
        settings.closed_ticks = Number(calibration["closed_ticks"].as<std::string>());
        if (calibration["homing_offset"]) {
            settings.gripper_offset = Integer(calibration, "homing_offset", -2047, 2047);
        }
        Require(settings.open_ticks >= 0 && settings.open_ticks <= 4095 &&
                        settings.closed_ticks >= 0 && settings.closed_ticks <= 4095 &&
                        std::abs(settings.open_ticks - settings.closed_ticks) >= 100,
                "gripper calibration requires two distinct encoder endpoints (at least 100 ticks)");
        settings.gripper_calibrated = true;
    }
    return settings;
}

DriverOptions Resolve(const DriverOptions& options, const std::string& directory) {
    auto result = options;
    for (const auto* key : {"port", "arm_calibration", "gripper_calibration"}) {
        auto found = result.find(key);
        if (found != result.end() && !found->second.empty()) {
            auto path = std::filesystem::path(found->second);
            if (path.is_relative()) path = std::filesystem::path(directory) / path;
            found->second = std::filesystem::weakly_canonical(path).string();
        }
    }
    return result;
}

SharedSession::SharedSession(Settings settings) : settings_(std::move(settings)) {}

SharedSession::~SharedSession() {
    feetech_bus_close(bus_);
}

void SharedSession::Connect() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!bus_) {
        std::vector<uint8_t> ids(settings_.ids.begin(), settings_.ids.end());
        if (settings_.gripper_calibrated) ids.push_back(settings_.gripper_id);
        bus_ = feetech_bus_open(settings_.port.c_str(), ids.data(), ids.size());
        if (!bus_) throw std::system_error(errno, std::generic_category(), "open SO101 serial bus");
    }
    ++owners_;
}

void SharedSession::Disconnect() noexcept {
    std::lock_guard<std::mutex> guard(mutex_);
    if (owners_ && --owners_ == 0) {
        feetech_bus_close(bus_);
        bus_ = nullptr;
    }
}

std::vector<uint8_t> SharedSession::Registers(uint8_t id, uint8_t address, size_t count) {
    Require(bus_ != nullptr, "not connected");
    std::vector<uint8_t> bytes(count);
    if (feetech_bus_read(bus_, id, address, bytes.data(), bytes.size()) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "read SO101 motor " + std::to_string(id));
    }
    return bytes;
}

Sample SharedSession::Read(uint8_t id) {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto data = Registers(id, 0, 71);  // Identity, configuration and feedback in one frame.
    Require(Word(data, 3) == 777, "motor " + std::to_string(id) + " is not an STS3215 (model 777)");
    Sample sample;
    sample.firmware = std::to_string(data[0]) + "." + std::to_string(data[1]);
    sample.minimum = Word(data, 9);
    sample.maximum = Word(data, 11);
    const int encoded_offset = Word(data, 31);
    Require((encoded_offset & 0xf000) == 0, "invalid homing offset register");
    sample.offset = (encoded_offset & 0x800) ? -(encoded_offset & 0x7ff) : encoded_offset;
    sample.position_mode = data[33] == 0;
    sample.enabled = data[40] != 0;
    sample.position = Word(data, 56);
    sample.moving = data[66] != 0;
    sample.error = data[65];
    sample.fault = sample.error != 0;
    const int load = Word(data, 60);  // Sign-magnitude, direction in bit 10.
    Require((load & ~0x7ff) == 0 && (load & 0x3ff) <= 1000, "invalid load register");
    sample.load = (load & 0x400) ? -(load & 0x3ff) : load;
    sample.torque_limit = Word(data, 16);
    sample.protection_current = Word(data, 28);
    Require(sample.minimum >= 0 && sample.maximum <= 4095 && sample.minimum < sample.maximum &&
                    sample.position <= 4095,
            "invalid position-mode limits or encoder feedback");
    return sample;
}

void SharedSession::Write(uint8_t id, uint8_t address, const std::vector<uint8_t>& bytes) {
    if (feetech_bus_write(bus_, id, address, bytes.data(), bytes.size()) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "register " + std::to_string(address) +
                                        " write was not confirmed for SO101 motor " +
                                        std::to_string(id));
    }
}

void SharedSession::Target(const std::vector<uint8_t>& ids, const std::vector<int>& ticks,
                           const std::vector<double>& velocities) {
    std::lock_guard<std::mutex> guard(mutex_);
    Require(bus_ != nullptr, "not connected");
    Require(ids.size() == ticks.size() && ids.size() == velocities.size(), "target size mismatch");
    for (size_t i = 0; i < ids.size(); ++i) {
        Require(ticks[i] >= 0 && ticks[i] <= 4095 && std::isfinite(velocities[i]) &&
                        velocities[i] >= kSpeedTickRadS && velocities[i] <= 3.6,
                "invalid motor target");
        // STS speed uses encoder ticks/second. Floor preserves the requested velocity ceiling.
        const int speed = static_cast<int>(std::floor(velocities[i] / kSpeedTickRadS));
        // SRAM goal position / time=0 / speed. Acceleration and EEPROM remain as installed.
        Write(ids[i], 42,
              {static_cast<uint8_t>(ticks[i] & 255), static_cast<uint8_t>(ticks[i] >> 8), 0, 0,
               static_cast<uint8_t>(speed & 255), static_cast<uint8_t>(speed >> 8)});
    }
}

void SharedSession::Torque(const std::vector<uint8_t>& ids, bool enabled) {
    std::lock_guard<std::mutex> guard(mutex_);
    Require(bus_ != nullptr, "not connected");
    for (const auto id : ids) Write(id, 40, {static_cast<uint8_t>(enabled ? 1 : 0)});
}

const Settings& SharedSession::Config() const {
    return settings_;
}
}  // namespace spacemit::manipulation::so101
