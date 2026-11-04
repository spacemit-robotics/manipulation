/* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * Explicit Feetech register transactions for the SO101 driver and so101-calibrate, through the
 * public API of the SDK motor component. Opening and closing never write servo registers.
 */
#ifndef FEETECH_BUS_H
#define FEETECH_BUS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct feetech_bus;

/** Take exclusive ownership of a serial port for the given servos, id 1..253 (1 Mbps, the only rate of
 * the motor driver). Returns NULL with errno on failure, EBUSY when another owner holds the port.
 */
struct feetech_bus *feetech_bus_open(const char *port, const uint8_t *ids, size_t count);

/** Release ownership; never changes torque. The motor driver keeps its descriptor until the process
 * exits, so a port that disappeared needs a new process. NULL is accepted.
 */
void feetech_bus_close(struct feetech_bus *bus);

/** Read registers of an opened servo, address + length <= 71. Returns 0 only for a complete,
 * checksum-verified response, otherwise -1 with errno. Servo errors are in the Status register (65).
 * The motor driver retries internally: a failure can take about 3 s when a response stays incomplete.
 */
int feetech_bus_read(struct feetech_bus *bus, uint8_t id, uint8_t address, uint8_t *data, size_t length);

/** Write registers one servo at a time and read them back: the motor driver does not report a lost
 * write. It unlocks EEPROM around writes to addresses 5..39. Returns 0 once confirmed, otherwise -1
 * with errno; a failure may follow partial delivery.
 */
int feetech_bus_write(struct feetech_bus *bus, uint8_t id, uint8_t address, const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
#endif  /* FEETECH_BUS_H */
