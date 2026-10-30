// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Socketpair avoids process-global SIGPIPE changes; spawn is safe for threaded callers.
#include "worker_channel.h"

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <thread>
#include <vector>

extern char** environ;

namespace spacemit::manipulation::agx {

WorkerChannel::~WorkerChannel() { Close(); }

void WorkerChannel::Start(const std::vector<std::string>& arguments) {
    if (socket_ >= 0 || arguments.empty()) throw std::runtime_error("invalid worker start");
    int pair[2];
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0, pair) != 0) {
        throw std::runtime_error("worker socketpair failed");
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, pair[1], STDIN_FILENO);
    posix_spawn_file_actions_adddup2(&actions, pair[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, pair[0]);
    posix_spawn_file_actions_addclose(&actions, pair[1]);
    // Terminal Ctrl+C and service stop signals go to the owner only: the worker must stay alive
    // to carry the hold. Its own group plus a blocked mask also covers interpreter startup.
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    sigset_t blocked;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGINT);
    sigaddset(&blocked, SIGTERM);
    posix_spawnattr_setsigmask(&attributes, &blocked);
    posix_spawnattr_setpgroup(&attributes, 0);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK);
    std::vector<char*> argv;
    for (const auto& argument : arguments) argv.push_back(const_cast<char*>(argument.c_str()));
    argv.push_back(nullptr);
    const int result = posix_spawnp(&pid_, argv[0], &actions, &attributes, argv.data(), environ);
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    close(pair[1]);
    if (result != 0) { close(pair[0]); pid_ = -1; throw std::runtime_error("worker spawn failed"); }
    socket_ = pair[0];
    request_ = 0;
    session_ = std::to_string(getpid()) + "-" + std::to_string(Clock::now().time_since_epoch().count());
}

void WorkerChannel::Wait(short events, Clock::time_point deadline) {
    while (true) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (remaining <= 0) throw std::runtime_error("worker RPC timed out; physical stop unknown");
        struct pollfd item {socket_, events, 0};
        const int result = poll(&item, 1, static_cast<int>(remaining));
        if (result < 0 && errno == EINTR) continue;
        if (result <= 0) throw std::runtime_error("worker RPC poll timeout/failure");
        if (item.revents & events) return;
        throw std::runtime_error("worker disconnected; physical stop unknown");
    }
}

YAML::Node WorkerChannel::Call(const std::string& operation, const std::string& fields) {
    if (socket_ < 0) throw std::runtime_error("worker not connected");
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline.time_since_epoch()).count();
    const auto id = ++request_;
    const std::string request = "{\"v\":1,\"session\":\"" + session_ + "\",\"id\":" + std::to_string(id) +
        ",\"deadline_ns\":" + std::to_string(nanoseconds) + ",\"op\":\"" + operation + "\"" + fields + "}\n";
    bool rejected = false;
    try {
        std::size_t offset = 0;
        while (offset < request.size()) {
            Wait(POLLOUT, deadline);
            const auto size = send(socket_, request.data() + offset, request.size() - offset, MSG_NOSIGNAL);
            if (size < 0 && (errno == EAGAIN || errno == EINTR)) continue;
            if (size <= 0) throw std::runtime_error("worker write failed");
            offset += static_cast<std::size_t>(size);
        }
        std::string response;
        while (response.empty() || response.back() != '\n') {
            Wait(POLLIN, deadline);
            char bytes[2048];
            const auto size = recv(socket_, bytes, sizeof(bytes), 0);
            if (size < 0 && (errno == EAGAIN || errno == EINTR)) continue;
            if (size <= 0) throw std::runtime_error("worker EOF; physical stop unknown");
            response.append(bytes, static_cast<std::size_t>(size));
            if (response.size() > 32768) throw std::runtime_error("worker response oversized");
        }
        if (response.find('\n') != response.size() - 1) throw std::runtime_error("unexpected worker frame");
        const auto node = YAML::Load(response);
        if (node["v"].as<int>() != 1 || node["session"].as<std::string>() != session_ ||
            node["id"].as<std::uint64_t>() != id) throw std::runtime_error("worker response identity mismatch");
        if (node["ok"].as<bool>()) return node["data"];
        rejected = node["rejected"] && node["rejected"].as<bool>();
        throw std::runtime_error(node["error"].as<std::string>());
    } catch (const std::exception& error) {
        // A well-formed refusal before any device write keeps the channel and its state.
        if (rejected) throw CommandRejected(error.what());
        Close();  // Otherwise the request may have executed; never resynchronize or assert a stop.
        throw;
    } catch (...) {
        Close();
        throw;
    }
}

void WorkerChannel::Close() noexcept {
    if (socket_ >= 0) { shutdown(socket_, SHUT_RDWR); close(socket_); socket_ = -1; }
    if (pid_ <= 0) return;
    for (int attempt = 0; attempt < 20; ++attempt) {
        const auto result = waitpid(pid_, nullptr, WNOHANG);
        if (result == pid_ || (result < 0 && errno == ECHILD)) { pid_ = -1; return; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(pid_, SIGKILL);
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
    pid_ = -1;
}
}  // namespace spacemit::manipulation::agx
