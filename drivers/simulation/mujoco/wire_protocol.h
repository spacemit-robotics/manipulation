// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Bounded length-prefixed frames. Socket ownership belongs to the caller.
#ifndef WIRE_PROTOCOL_H
#define WIRE_PROTOCOL_H

#include <yaml-cpp/yaml.h>

#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "session_backend.h"

namespace spacemit::manipulation::simulation::wire {
constexpr std::size_t kControlLimit = 65536;
constexpr std::size_t kImageLimit = 32 * 1024 * 1024;
std::string Quote(const std::string& value);
std::string Number(double value);
std::string Array(const std::vector<double>& values);
std::string Strings(const std::vector<std::string>& values);
std::string Object(std::initializer_list<std::pair<std::string, std::string>> fields);
void Keys(const YAML::Node& node, std::initializer_list<const char*> allowed);
double Finite(const YAML::Node& node);
std::uint64_t Unsigned(const YAML::Node& node);
std::vector<double> Numbers(const YAML::Node& node, std::size_t count);
YAML::Node Parse(const std::string& frame);
std::string Nonce();
int ConnectLoopback(const TransportOptions& options, Clock::time_point deadline);
std::string ReadFrame(int socket, Clock::time_point deadline, std::size_t limit = kControlLimit);
void WriteFrame(int socket, const std::string& payload, Clock::time_point deadline,
        std::size_t limit = kControlLimit);
}  // namespace spacemit::manipulation::simulation::wire
#endif  // WIRE_PROTOCOL_H
