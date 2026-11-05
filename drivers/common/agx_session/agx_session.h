// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Private, serialized, reference-counted transport for arm and effector drivers.
#ifndef MANIPULATION_DRIVERS_COMMON_AGX_SESSION_AGX_SESSION_H_
#define MANIPULATION_DRIVERS_COMMON_AGX_SESSION_AGX_SESSION_H_
#include <memory>
#include <string>
#include <utility>
#include "manipulation/device.h"
#include "worker_channel.h"
namespace spacemit::manipulation::agx {
class SharedSession {
public:
    explicit SharedSession(DriverOptions options) : options_(std::move(options)) {}
    void Acquire() {
        if (owners_ == 0) {
            worker_.Start({options_.at("python"), "-I", "-u", options_.at("worker"),
                "--sdk-root", options_.at("sdk_root"), "--channel", options_.at("channel")});
            try { worker_.Call("connect"); } catch (...) { worker_.Close(); throw; }
        }
        ++owners_;
    }
    void Release() noexcept { if (owners_ > 0 && --owners_ == 0) worker_.Close(); }
    YAML::Node Call(const std::string& operation, const std::string& fields = "") {
        return worker_.Call(operation, fields);
    }
private:
    DriverOptions options_;
    WorkerChannel worker_;
    unsigned owners_ = 0;
};
void ValidateArm(const DeviceConfig& config, const DriverOptions& options);
DriverOptions ResolveOptions(const DriverOptions& options, const std::string& directory);
std::unique_ptr<Arm> MakeArm(const DeviceConfig& config, const DriverOptions& options,
    std::shared_ptr<SharedSession> connection);
std::unique_ptr<EndEffector> MakeEffector(const EffectorConfig& config, std::shared_ptr<SharedSession> connection);
}  // namespace spacemit::manipulation::agx
#endif  // MANIPULATION_DRIVERS_COMMON_AGX_SESSION_AGX_SESSION_H_
