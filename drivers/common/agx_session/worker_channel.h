// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Private bounded local worker protocol; never exported to applications.
#ifndef WORKER_CHANNEL_H
#define WORKER_CHANNEL_H

#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <string>
#include <vector>

#include "manipulation/types.h"

namespace spacemit::manipulation::agx {

class WorkerChannel {
public:
    ~WorkerChannel();
    void Start(const std::vector<std::string>& arguments);
    YAML::Node Call(const std::string& operation, const std::string& fields = "");
    void Close() noexcept;

private:
    void Wait(short events, Clock::time_point deadline);
    int socket_ = -1;
    pid_t pid_ = -1;
    std::uint64_t request_ = 0;
    std::string session_;
};

}  // namespace spacemit::manipulation::agx
#endif  // WORKER_CHANNEL_H
