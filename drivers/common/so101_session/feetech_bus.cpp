// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
// Register access through the motor component's Feetech driver. motor_init is deliberately not called:
// it rewrites the EEPROM mode on every connection, and a connection here must only read.
#include "feetech_bus.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include "motor.h"

namespace {
constexpr char kDriver[] = "drv_uart_feetech";
constexpr uint32_t kBaud = 1000000;
constexpr size_t kRegisters = 71;  // The driver's address space.

// The driver prints while it opens the port, and this process's stdout may be a protocol stream.
class StdoutToStderr {
public:
    StdoutToStderr() {
        Flush();
        saved_ = fcntl(STDOUT_FILENO, F_DUPFD_CLOEXEC, 3);
        if (saved_ >= 0) dup2(STDERR_FILENO, STDOUT_FILENO);
    }
    ~StdoutToStderr() {
        Flush();
        if (saved_ < 0) return;
        dup2(saved_, STDOUT_FILENO);
        close(saved_);
    }
    StdoutToStderr(const StdoutToStderr&) = delete;
    StdoutToStderr& operator=(const StdoutToStderr&) = delete;
private:
    static void Flush() {
        std::cout.flush();
        std::fflush(stdout);
    }
    int saved_ = -1;
};
}  // namespace

struct feetech_bus {
    int guard = -1;  // Holds the exclusion; the driver's own descriptor is private to it.
    bool exclusive = false;
    std::map<uint8_t, motor_dev*> servos;
};

namespace {
motor_dev* Servo(feetech_bus* bus, uint8_t id, uint8_t address, const void* data, size_t length) {
    if (bus && data && length > 0 && address + length <= kRegisters) {
        const auto found = bus->servos.find(id);
        if (found != bus->servos.end()) return found->second;
    }
    errno = EINVAL;
    return nullptr;
}
}  // namespace

extern "C" {
void feetech_bus_close(feetech_bus* bus) {
    if (!bus) return;
    for (auto& entry : bus->servos) motor_free(&entry.second, 1);
    // The driver's descriptor stays open, so exclusive mode would otherwise outlive this owner.
    if (bus->exclusive) ioctl(bus->guard, TIOCNXCL);
    close(bus->guard);
    delete bus;
}

feetech_bus* feetech_bus_open(const char* port, const uint8_t* ids, size_t count) {
    if (!port || !ids || count == 0) {
        errno = EINVAL;
        return nullptr;
    }
    const int guard = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (guard < 0) return nullptr;
    if (flock(guard, LOCK_EX | LOCK_NB) != 0) {
        const int error = errno == EWOULDBLOCK ? EBUSY : errno;
        close(guard);
        errno = error;
        return nullptr;
    }
    auto* bus = new feetech_bus;
    bus->guard = guard;
    int error = 0;
    {
        StdoutToStderr quiet;
        for (size_t i = 0; i < count && !error; ++i) {
            motor_dev* servo = ids[i] >= 1 && ids[i] <= 253 && !bus->servos.count(ids[i]) ?
                motor_alloc_uart(kDriver, port, kBaud, ids[i], nullptr) : nullptr;
            if (servo) {
                bus->servos[ids[i]] = servo;
            } else {
                error = ENODEV;
            }
        }
    }
    // Only after the driver has opened the port: exclusive mode would also refuse the driver.
    if (!error) {
        if (ioctl(guard, TIOCEXCL) == 0) {
            bus->exclusive = true;
        } else {
            error = errno;
        }
    }
    if (error) {
        feetech_bus_close(bus);
        errno = error;
        return nullptr;
    }
    return bus;
}

int feetech_bus_read(feetech_bus* bus, uint8_t id, uint8_t address, uint8_t* data, size_t length) {
    motor_dev* servo = Servo(bus, id, address, data, length);
    if (!servo) return -1;
    if (motor_get_paras(servo, &address, data, static_cast<uint32_t>(length)) != 0) {
        errno = EIO;
        return -1;
    }
    return 0;
}

int feetech_bus_write(feetech_bus* bus, uint8_t id, uint8_t address, const uint8_t* data, size_t length) {
    motor_dev* servo = Servo(bus, id, address, data, length);
    if (!servo) return -1;
    std::vector<uint8_t> actual(length);
    if (motor_set_paras(servo, &address, data, static_cast<uint32_t>(length)) != 0 ||
        motor_get_paras(servo, &address, actual.data(), static_cast<uint32_t>(length)) != 0 ||
        std::memcmp(actual.data(), data, length) != 0) {
        errno = EIO;
        return -1;
    }
    return 0;
}
}  // extern "C"
