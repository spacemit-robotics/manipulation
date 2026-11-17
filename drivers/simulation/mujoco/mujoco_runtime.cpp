// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Local embedding facade; rendered observations and public roles share one physics owner.
#include "manipulation/mujoco_runtime.h"

#include <stdexcept>

#include "mujoco_session.h"

namespace spacemit::manipulation {
class MujocoRuntime::Impl {
public:
    std::shared_ptr<simulation::SharedSession> session;
    bool effector = false;
    bool started = false;
};

MujocoRuntime::MujocoRuntime(const DeviceDefinition& definition, const DriverOptions& options)
    : impl_(std::make_unique<Impl>()) {
    if (simulation::ParseTransport(options).remote)
        throw std::runtime_error("embedded MuJoCo runtime requires local transport");
    ValidateDevice(definition.arm);
    ValidateEffector(definition.effector);
    impl_->effector = !definition.effector.channels.empty();
    impl_->session = std::make_shared<simulation::SharedSession>(simulation::LoadMapping(options),
            definition.arm,
            impl_->effector ? std::optional<EffectorConfig>(definition.effector) : std::nullopt);
}

MujocoRuntime::~MujocoRuntime() {
    Stop();
}

void MujocoRuntime::Start() {
    if (impl_->started) throw std::runtime_error("MuJoCo runtime already started");
    impl_->session->Acquire();
    impl_->started = true;
}

void MujocoRuntime::Stop() noexcept {
    if (impl_->started) impl_->session->Release();
    impl_->started = false;
}

void MujocoRuntime::Reset() {
    if (!impl_->started) throw std::runtime_error("MuJoCo runtime not started");
    impl_->session->Reset();
}

bool MujocoRuntime::Hold() {
    if (!impl_->started) throw std::runtime_error("MuJoCo runtime not started");
    return impl_->session->HoldAll();
}

Devices MujocoRuntime::CreateDevices() {
    return simulation::WrapSession(impl_->session, true, impl_->effector);
}

mujoco_sim::WorldSnapshot MujocoRuntime::Snapshot() {
    return impl_->session->Snapshot();
}

void MujocoRuntime::AttachWeld(const std::string& equality, const std::string& body1,
        const std::string& body2, const std::string& left_pad, const std::string& right_pad) {
    impl_->session->AttachWeld(equality, body1, body2, left_pad, right_pad);
}

void MujocoRuntime::ReleaseWeld(const std::string& equality) {
    impl_->session->ReleaseWeld(equality);
}
}  // namespace spacemit::manipulation
