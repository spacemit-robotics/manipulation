// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "manipulation/kinematics.h"

#include <stdexcept>
#include <utility>

namespace spacemit::manipulation {

void KinematicsRegistry::Register(const std::string& id, Factory factory) {
    if (id.empty() || !factory) throw std::invalid_argument("invalid kinematics registration");
    if (!factories_.emplace(id, std::move(factory)).second)
        throw std::invalid_argument("duplicate kinematics backend: " + id);
}

std::unique_ptr<Kinematics> KinematicsRegistry::Create(const std::string& id,
        const KinematicsConfig& config) const {
    const auto found = factories_.find(id);
    if (found == factories_.end()) throw std::invalid_argument("unknown kinematics backend: " + id);
    auto result = found->second(config);
    if (!result) throw std::runtime_error("kinematics factory returned no solver: " + id);
    return result;
}

}  // namespace spacemit::manipulation
