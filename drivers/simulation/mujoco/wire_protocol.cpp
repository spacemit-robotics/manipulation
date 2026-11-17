// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Every connect/read/write uses a caller-owned deadline; incomplete frames are never dispatched.
#include "wire_protocol.h"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace spacemit::manipulation::simulation::wire {
namespace {
void Wait(int socket, short events, Clock::time_point deadline) {
    for (;;) {
        const auto left =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now())
                        .count();
        if (left <= 0) throw std::runtime_error("simulation transport deadline; stop unknown");

        struct pollfd item {
            socket, events, 0
        };

        const auto result = poll(&item, 1, static_cast<int>(std::min<std::int64_t>(left, 1000)));
        if (result < 0 && errno == EINTR) continue;
        if (result < 0) throw std::runtime_error("simulation transport poll failed");
        if (!result) continue;
        if (item.revents & events) return;
        throw std::runtime_error("simulation transport disconnected; stop unknown");
    }
}

void Transfer(int socket, char* bytes, std::size_t size, Clock::time_point deadline, bool sending) {
    std::size_t offset = 0;
    while (offset < size) {
        Wait(socket, sending ? POLLOUT : POLLIN, deadline);
        const auto count = sending
                ? send(socket, bytes + offset, size - offset, MSG_NOSIGNAL | MSG_DONTWAIT)
                : recv(socket, bytes + offset, size - offset, MSG_DONTWAIT);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (count <= 0)
            throw std::runtime_error("simulation frame EOF/write failure; stop unknown");
        offset += static_cast<std::size_t>(count);
    }
}
}  // namespace

std::string Quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (const unsigned char byte : value) {
        if (byte == '"' || byte == '\\')
            out << '\\' << byte;
        else if (byte < 0x20)
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<unsigned>(byte);
        else
            out << byte;
    }
    out << '"';
    return out.str();
}

std::string Number(double value) {
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite simulation protocol number");
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return out.str();
}

std::string Array(const std::vector<double>& values) {
    std::string result = "[";
    for (const auto value : values) {
        if (result.size() > 1) result += ',';
        result += Number(value);
    }
    return result + ']';
}

std::string Strings(const std::vector<std::string>& values) {
    std::string result = "[";
    for (const auto& value : values) {
        if (result.size() > 1) result += ',';
        result += Quote(value);
    }
    return result + ']';
}

std::string Object(std::initializer_list<std::pair<std::string, std::string>> fields) {
    std::string result = "{";
    for (const auto& [name, value] : fields) {
        if (result.size() > 1) result += ',';
        result += Quote(name) + ':' + value;
    }
    return result + '}';
}

void Keys(const YAML::Node& node, std::initializer_list<const char*> allowed) {
    if (!node.IsMap()) throw std::runtime_error("simulation protocol expects an object");
    std::set<std::string> seen;
    for (const auto& entry : node) {
        const auto name = entry.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), name) == allowed.end() ||
                !seen.insert(name).second)
            throw std::runtime_error("unknown/duplicate protocol key");
    }
}

double Finite(const YAML::Node& node) {
    const auto value = node.as<double>();
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite simulation protocol number");
    return value;
}

std::uint64_t Unsigned(const YAML::Node& node) {
    const auto value = node.as<std::string>();
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("invalid unsigned protocol integer");
    return node.as<std::uint64_t>();
}

std::vector<double> Numbers(const YAML::Node& node, std::size_t count) {
    if (!node.IsSequence() || node.size() != count)
        throw std::runtime_error("simulation protocol dimensions mismatch");
    std::vector<double> result;
    for (const auto& value : node) result.push_back(Finite(value));
    return result;
}

YAML::Node Parse(const std::string& frame) {
    if (frame.empty() || frame.front() != '{' || frame.back() != '}' ||
            frame.find('\0') != std::string::npos)
        throw std::runtime_error("invalid simulation JSON frame");
    // yaml-cpp is already a dependency, but wire data must not gain YAML aliases/tags or
    // unbounded nesting. They are not part of this protocol and can amplify a tiny frame.
    unsigned depth = 0;
    bool quoted = false;
    for (std::size_t index = 0; index < frame.size(); ++index) {
        const unsigned char byte = frame[index];
        if (quoted) {
            if (byte < 0x20)
                throw std::runtime_error("control character in simulation JSON string");
            if (byte == '\\') {
                if (++index >= frame.size() ||
                        std::string("\"\\/bfnrtu").find(frame[index]) == std::string::npos)
                    throw std::runtime_error("invalid simulation JSON escape");
            } else if (byte == '"')
                quoted = false;
        } else if (byte == '"')
            quoted = true;
        else if (byte == '{' || byte == '[') {
            if (++depth > 32) throw std::runtime_error("simulation JSON nesting exceeds limit");
        } else if (byte == '}' || byte == ']') {
            if (!depth) throw std::runtime_error("unbalanced simulation JSON");
            --depth;
        } else if (std::string(" \t\r\n,:-0123456789.eE+truefalsn").find(byte) ==
                std::string::npos) {
            throw std::runtime_error("non-JSON simulation protocol token");
        }
    }
    if (quoted || depth) throw std::runtime_error("incomplete simulation JSON");
    const auto documents = YAML::LoadAll(frame);
    if (documents.size() != 1 || !documents.front().IsMap())
        throw std::runtime_error("invalid simulation frame document");
    return documents.front();
}

std::string Nonce() {
    unsigned char bytes[16];
    std::size_t offset = 0;
    while (offset < sizeof(bytes)) {
        const auto size = getrandom(bytes + offset, sizeof(bytes) - offset, 0);
        if (size < 0 && errno == EINTR) continue;
        if (size <= 0) throw std::runtime_error("cannot create simulation session identity");
        offset += static_cast<std::size_t>(size);
    }
    std::ostringstream out;
    for (const auto byte : bytes)
        out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
    return out.str();
}

int ConnectLoopback(const TransportOptions& options, Clock::time_point deadline) {
    if ((options.host != "127.0.0.1" && options.host != "localhost") || options.port == 0 ||
            options.port > 65535)
        throw std::invalid_argument("simulation TCP requires a loopback endpoint");
    const int socket = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket < 0) throw std::runtime_error("cannot create simulation socket");
    try {
        const int no_delay = 1;
        if (setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)))
            throw std::runtime_error("cannot configure simulation socket");

        struct sockaddr_in address {};

        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<std::uint16_t>(options.port));
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (connect(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            if (errno != EINPROGRESS) throw std::runtime_error("simulation connect failed");
            Wait(socket, POLLOUT, deadline);
            int error = 0;
            socklen_t length = sizeof(error);
            if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &error, &length) || error)
                throw std::runtime_error("simulation connect failed");
        }
        return socket;
    } catch (...) {
        close(socket);
        throw;
    }
}

std::string ReadFrame(int socket, Clock::time_point deadline, std::size_t limit) {
    std::uint32_t length = 0;
    Transfer(socket, reinterpret_cast<char*>(&length), sizeof(length), deadline, false);
    length = ntohl(length);
    if (!length || length > limit) throw std::runtime_error("simulation frame length rejected");
    std::string payload(length, '\0');
    Transfer(socket, payload.data(), payload.size(), deadline, false);
    return payload;
}

void WriteFrame(
        int socket, const std::string& payload, Clock::time_point deadline, std::size_t limit) {
    if (payload.empty() || payload.size() > limit)
        throw std::runtime_error("simulation frame length rejected");
    auto length = htonl(static_cast<std::uint32_t>(payload.size()));
    Transfer(socket, reinterpret_cast<char*>(&length), sizeof(length), deadline, true);
    Transfer(socket, const_cast<char*>(payload.data()), payload.size(), deadline, true);
}
}  // namespace spacemit::manipulation::simulation::wire
