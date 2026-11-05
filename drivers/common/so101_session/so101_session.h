// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// SO101 composition and calibrated register access; private to the registered driver.
#ifndef SO101_SESSION_H
#define SO101_SESSION_H

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/device.h"

struct feetech_bus;

namespace spacemit::manipulation::so101 {
constexpr size_t kArmJoints = 5;
constexpr double kSpeedTickRadS = 6.28318530717958647692 / 4096.0;
// Status bit 5: overload protection is active and output is limited to Overload_Torque.
constexpr uint8_t kOverloadError = 0x20;
// Gripper protection written by `so101-calibrate configure` (LeRobot's values): enabling an
// unconfigured gripper could stall it at full torque.
constexpr int kGripperTorqueLimit = 500;          // Max_Torque_Limit, 0.1 % of rated torque
constexpr int kGripperProtectionCurrent = 250;    // Protection_Current, mA
struct Calibration {
    int offset = 0;
    int minimum = 0;
    int maximum = 4095;
};
struct Settings {
    std::string port;
    std::array<uint8_t, kArmJoints> ids{{1, 2, 3, 4, 5}};
    uint8_t gripper_id = 6;
    double maximum_velocity = 1.0;
    bool arm_calibrated = false;
    std::array<Calibration, kArmJoints> calibration{};
    bool gripper_calibrated = false;
    double closed_ticks = 0;
    double open_ticks = 0;
    std::optional<int> gripper_offset;
    int grip_load_threshold = 100;  // |Present_Load| (0.1 % of rated torque) that counts as holding
};
struct Sample {
    int position = 0;
    int offset = 0;
    int minimum = 0;
    int maximum = 4095;
    bool enabled = false;
    bool position_mode = false;
    bool moving = false;
    bool fault = false;      // Any servo error in the Status register.
    uint8_t error = 0;       // The error bits themselves.
    int load = 0;            // Present_Load, signed, 0.1 % of rated torque.
    int torque_limit = 0;    // Max_Torque_Limit
    int protection_current = 0;
    std::string firmware;
};
Settings Parse(const DeviceDefinition& definition, const DriverOptions& options);
DriverOptions Resolve(const DriverOptions& options, const std::string& directory);
double JointPosition(int ticks);
int JointTicks(double radians);

class SharedSession {
public:
    explicit SharedSession(Settings settings);
    ~SharedSession();
    void Connect();
    void Disconnect() noexcept;
    Sample Read(uint8_t id);
    void Target(const std::vector<uint8_t>& ids, const std::vector<int>& ticks,
        const std::vector<double>& velocities);
    void Torque(const std::vector<uint8_t>& ids, bool enabled);
    const Settings& Config() const;
private:
    std::vector<uint8_t> Registers(uint8_t id, uint8_t address, size_t count);
    void Write(uint8_t id, uint8_t address, const std::vector<uint8_t>& bytes);
    Settings settings_;
    feetech_bus* bus_ = nullptr;
    size_t owners_ = 0;
    std::mutex mutex_;
};
std::unique_ptr<Arm> MakeArm(const DeviceConfig& definition, std::shared_ptr<SharedSession> session);
std::unique_ptr<EndEffector> MakeGripper(const EffectorConfig& definition,
    std::shared_ptr<SharedSession> session);
}  // namespace spacemit::manipulation::so101
#endif  // SO101_SESSION_H
