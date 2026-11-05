#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""SO101 servo setup with torque released: register status, parameter configuration, mid-position
(homing offset) calibration, travel capture and servo limits, or all of them in order. Never enables
torque.

The calibration files live in SITE/so101/, beside the SO101 device configuration that
`manipulation configure so101` creates there, under the names its templates use.

Only configure, center and limits write servo EEPROM: the motor driver unlocks and relocks it around
each value, which is then read back; values already equal are not written.
"""

import argparse
import contextlib
import ctypes
import json
import os
from pathlib import Path
import select
import sys
import termios
import time
import time

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MIDDLE = 2047  # Encoder ticks of the model zero pose (0 rad) after center.
# Turns past half a turn, so a captured travel could wrap around the encoder: it keeps the whole
# circle, as in LeRobot, and the configured joint limits bound it.
FULL_TURN = "wrist_roll"
SWEEP_PERIOD_S = 0.02  # Pause between two readings of the live travel table.
ARM_FILE, GRIPPER_FILE = "so101_calibration.json", "so101_gripper_calibration.json"
LOCK = 55  # Reported by status; the motor driver handles the EEPROM lock around writes.
# (address, length, value) recommended for the SO101 (LeRobot's values):
# return delay 0, P 16, D 0, I 0, position mode; the gripper also limits torque and current.
ARM_PARAMETERS = ((7, 1, 0), (21, 1, 16), (22, 1, 0), (23, 1, 0), (33, 1, 0))
GRIPPER_PARAMETERS = ARM_PARAMETERS + ((16, 2, 500), (28, 2, 250), (36, 1, 25))


class RegisterBus:
    """Binding to the SDK's exclusively owned Feetech register bus."""

    def __init__(self, library, port, motor_ids):
        self.library = ctypes.CDLL(str(library), use_errno=True)
        self.library.feetech_bus_open.argtypes = [
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        ]
        self.library.feetech_bus_open.restype = ctypes.c_void_p
        self.library.feetech_bus_close.argtypes = [ctypes.c_void_p]
        self.library.feetech_bus_close.restype = None
        self.library.feetech_bus_read.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
        ]
        self.library.feetech_bus_read.restype = ctypes.c_int
        self.library.feetech_bus_write.argtypes = self.library.feetech_bus_read.argtypes
        self.library.feetech_bus_write.restype = ctypes.c_int
        ids = (ctypes.c_uint8 * len(motor_ids))(*motor_ids)
        self.handle = self.library.feetech_bus_open(os.fsencode(port), ids, len(motor_ids))
        if not self.handle:
            error = ctypes.get_errno()
            raise OSError(error, os.strerror(error), port)

    def registers(self, motor_id, address, length):
        data = (ctypes.c_uint8 * length)()
        if self.library.feetech_bus_read(self.handle, motor_id, address, data, length) != 0:
            error = ctypes.get_errno()
            raise OSError(error, f"servo {motor_id}: {os.strerror(error)}")
        return bytes(data)

    def read(self, motor_id, address, length):
        return int.from_bytes(self.registers(motor_id, address, length), "little")

    def write(self, motor_id, address, length, value):
        data = (ctypes.c_uint8 * length)(*value.to_bytes(length, "little"))
        if self.library.feetech_bus_write(self.handle, motor_id, address, data, length) != 0:
            error = ctypes.get_errno()
            raise OSError(
                error,
                f"servo {motor_id} register {address} write not confirmed: {os.strerror(error)}",
            )

    def close(self):
        if self.handle:
            self.library.feetech_bus_close(self.handle)
            self.handle = None


def library_path(explicit=None):
    """Resolve the library from an installation manifest, including a relocated tree."""
    if explicit:
        return Path(explicit).resolve(strict=True)
    for directory in Path(__file__).resolve().parents:
        manifest = directory / ".spacemit-manipulation.json"
        if manifest.is_file():
            layout = json.loads(manifest.read_text(encoding="utf-8"))
            return (directory / layout["libdir"] / "libmanipulation_feetech_bus.so").resolve(
                strict=True
            )
    raise ValueError("use the installed so101-calibrate command, or pass --library LIBRARY")


def decode_offset(value):
    """Homing offset register: sign-magnitude, sign in bit 11."""
    if value & ~0x0FFF:
        raise ValueError("invalid homing offset")
    return -(value & 0x07FF) if value & 0x0800 else value


def encode_offset(value):
    if not -2047 <= value <= 2047:
        raise ValueError("homing offset must be within +-2047 ticks")
    return 0x0800 | -value if value < 0 else value


def offset(bus, motor_id):
    try:
        return decode_offset(bus.read(motor_id, 31, 2))
    except ValueError:
        raise ValueError(f"servo {motor_id}: invalid homing offset") from None


def check_servos(bus, motor_ids, offsets=None, position_mode=True):
    """Never release torque here: the operator must support and prepare the arm first."""
    for motor_id in motor_ids:
        if bus.read(motor_id, 3, 2) != 777:
            raise ValueError(f"servo {motor_id}: expected STS3215 model 777")
        if bus.read(motor_id, 65, 1) != 0:
            raise ValueError(f"servo {motor_id} reports error {bus.read(motor_id, 65, 1)}")
        if bus.read(motor_id, 40, 1) != 0:
            raise ValueError(f"servo {motor_id}: torque must already be off; support the arm first")
        if position_mode and bus.read(motor_id, 33, 1) != 0:
            raise ValueError(f"servo {motor_id}: position mode required")
        if offsets is not None and offset(bus, motor_id) != offsets[motor_id]:
            raise ValueError(f"servo {motor_id}: homing offset changed during capture")


def present(bus, motor_id):
    value = bus.read(motor_id, 56, 2)
    if not 0 <= value <= 4095:
        raise ValueError(f"servo {motor_id}: invalid position {value}")
    return value


def enter_pressed():
    """True once the operator has pressed Enter; never blocks, as LeRobot's."""
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


@contextlib.contextmanager
def unechoed_input():
    """Keep the terminal from echoing keys while the live table is drawn: an echoed Enter would
    move the cursor and leave a stray table line behind."""
    if not sys.stdin.isatty():
        yield
        return
    descriptor = sys.stdin.fileno()
    saved = termios.tcgetattr(descriptor)
    quiet = termios.tcgetattr(descriptor)
    quiet[3] &= ~termios.ECHO
    termios.tcsetattr(descriptor, termios.TCSANOW, quiet)
    try:
        yield
    finally:
        termios.tcsetattr(descriptor, termios.TCSANOW, saved)


def travel_table(names, minimums, positions, maximums):
    """The LeRobot range-of-motion table: lowest, current and highest reading per joint."""
    lines = ["", "-" * 43, f"{'NAME':<15} | {'MIN':>6} | {'POS':>6} | {'MAX':>6}"]
    lines += [
        f"{name:<15} | {minimums[name]:>6} | {positions[name]:>6} | {maximums[name]:>6}"
        for name in names
    ]
    return "\n".join(lines) + "\n"


class LiveTable:
    """Redraws the travel table in place on a terminal; elsewhere prints only the final table."""

    def __init__(self, stream=None):
        self.stream, self.drawn = stream or sys.stdout, 0

    def __call__(self, text, final=False):
        if not self.stream.isatty():
            if final:
                self.stream.write(text)
            return
        if final:  # Already on screen; Enter moved the cursor below it.
            return
        if self.drawn:
            self.stream.write(f"\033[{self.drawn}A")
        self.stream.write(text)
        self.stream.flush()
        self.drawn = text.count("\n")


def sweep(bus, joints, enter_pressed=enter_pressed, show=None, period_s=SWEEP_PERIOD_S):
    """Record each joint's lowest and highest position, from the current pose, until Enter.

    joints maps names to servo IDs. A live table shows MIN, POS and MAX per joint. Returns the
    minimums, the maximums and the joints whose reading jumped by more than half the encoder
    between two samples: it wrapped around the encoder.
    """
    show = show or LiveTable()
    previous = {name: present(bus, motor_id) for name, motor_id in joints.items()}
    minimums, maximums, wrapped = dict(previous), dict(previous), set()
    while True:
        positions = {name: present(bus, motor_id) for name, motor_id in joints.items()}
        for name, value in positions.items():
            if abs(value - previous[name]) > 2048:
                wrapped.add(name)
            minimums[name] = min(minimums[name], value)
            maximums[name] = max(maximums[name], value)
        previous = positions
        show(travel_table(joints, minimums, positions, maximums))
        if enter_pressed():
            break
        time.sleep(period_s)
    show(travel_table(joints, minimums, positions, maximums), final=True)
    return minimums, maximums, wrapped


def stationary(bus, motor_id):
    first, second = bus.read(motor_id, 56, 2), bus.read(motor_id, 56, 2)
    if not 0 <= first <= 4095 or not 0 <= second <= 4095 or abs(first - second) > 3:
        raise ValueError(f"servo {motor_id}: position is invalid or not stationary")
    return second


def write_eeprom(bus, motor_id, values):
    """Write the (address, length, value) entries that differ; the bus reads each one back.

    Torque must already be off and stays off. Returns the entries written.
    """
    check_servos(bus, [motor_id], position_mode=False)
    changes = [
        (address, length, value)
        for address, length, value in values
        if bus.read(motor_id, address, length) != value
    ]
    for address, length, value in changes:
        bus.write(motor_id, address, length, value)
    return changes


def status(bus, motor_ids):
    """One read of registers 0..70 per servo, decoded; writes nothing."""
    result = {}
    for motor_id in motor_ids:
        data = bus.registers(motor_id, 0, 71)

        def word(address):
            return data[address] | data[address + 1] << 8

        load = word(60)
        result[motor_id] = {
            "model": word(3),
            "firmware": f"{data[0]}.{data[1]}",
            "mode": data[33],
            "torque_enabled": data[40],
            "eeprom_locked": data[LOCK],
            "homing_offset": decode_offset(word(31)),
            "range_min": word(9),
            "range_max": word(11),
            "p": data[21],
            "d": data[22],
            "i": data[23],
            "return_delay": data[7],
            "max_torque": word(16),
            "protection_current_ma": word(28),
            "overload_torque": data[36],
            "acceleration": data[41],
            "position": word(56),
            "load": -(load & 0x3FF) if load & 0x400 else load,
            "voltage_v": data[62] / 10,
            "temperature_c": data[63],
            "error": data[65],
            "moving": data[66],
        }
    return result


def configure(bus, arm_ids, gripper_id):
    """Recommended EEPROM parameters; returns {motor_id: [(address, length, value) written]}."""
    ids = list(arm_ids) + ([gripper_id] if gripper_id is not None else [])
    check_servos(bus, ids, position_mode=False)  # Mode is one of the parameters.
    return {
        motor_id: write_eeprom(
            bus, motor_id, GRIPPER_PARAMETERS if motor_id == gripper_id else ARM_PARAMETERS
        )
        for motor_id in ids
    }


def center(bus, motor_ids, prompt=input, settle_s=0.1):
    """Make the held pose the encoder middle: offset = position with offset 0, minus MIDDLE.

    Limits are reset to the full 0..4095 range, as they are in the old offset frame; capture new
    travel afterwards. Returns {motor_id: homing_offset}.
    """
    check_servos(bus, motor_ids)
    prompt(
        "Support the arm and hold it in the model zero pose, every joint near the middle of its"
        " travel."
        " Press Enter to calibrate, Ctrl+C to cancel: "
    )
    check_servos(bus, motor_ids)
    offsets = {}
    for motor_id in motor_ids:
        write_eeprom(bus, motor_id, ((31, 2, 0), (9, 2, 0), (11, 2, 4095)))
        time.sleep(settle_s)
        value = stationary(bus, motor_id) - MIDDLE
        if not -2047 <= value <= 2047:
            raise ValueError(
                f"servo {motor_id}: at the encoder end; move it towards the middle of its travel"
            )
        write_eeprom(bus, motor_id, ((31, 2, encode_offset(value)),))
        time.sleep(settle_s)
        if abs(stationary(bus, motor_id) - MIDDLE) > 3:
            raise ValueError(
                f"servo {motor_id}: position after calibration is not the middle; "
                "keep the arm still"
            )
        offsets[motor_id] = value
    return offsets


def limits(bus, arm, gripper):
    """Install captured travel as the servo position limits (registers 9/11), in the captured offset
    frame."""
    ranges = [
        (joint["id"], joint["homing_offset"], joint["range_min"], joint["range_max"])
        for joint in (arm or {}).values()
    ]
    if gripper:
        ranges.append(
            (
                gripper["motor_id"],
                gripper.get("homing_offset"),
                min(gripper["open_ticks"], gripper["closed_ticks"]),
                max(gripper["open_ticks"], gripper["closed_ticks"]),
            )
        )
    check_servos(bus, [motor_id for motor_id, *_ in ranges])
    for motor_id, homing, minimum, maximum in ranges:
        if not 0 <= minimum < maximum <= 4095:
            raise ValueError(f"servo {motor_id}: invalid captured range")
        if homing is not None and offset(bus, motor_id) != homing:
            raise ValueError(
                f"servo {motor_id}: homing offset differs from the calibration; capture again"
            )
        if not minimum <= stationary(bus, motor_id) <= maximum:
            raise ValueError(f"servo {motor_id}: move it into its captured range first")
    return {
        motor_id: write_eeprom(bus, motor_id, ((9, 2, minimum), (11, 2, maximum)))
        for motor_id, _, minimum, maximum in ranges
    }


def calibrate_all(
    bus,
    arm_ids,
    gripper_id,
    save,
    prompt=input,
    settle_s=0.1,
    enter_pressed=enter_pressed,
    show=None,
    period_s=SWEEP_PERIOD_S,
):
    """configure, center, capture and limits in order: the full setup of a new or reassembled arm.

    save(arm, gripper) stores the captured travel before it is written as servo limits.
    """
    ids = list(arm_ids) + ([gripper_id] if gripper_id is not None else [])
    configure(bus, arm_ids, gripper_id)
    center(bus, ids, prompt, settle_s)
    arm, gripper = capture(bus, arm_ids, gripper_id, prompt, enter_pressed, show, period_s)
    save(arm, gripper)
    limits(bus, arm, gripper)
    return arm, gripper


def capture(
    bus,
    arm_ids,
    gripper_id,
    prompt=input,
    enter_pressed=enter_pressed,
    show=None,
    period_s=SWEEP_PERIOD_S,
):
    """Capture the travel of the joints and the gripper endpoints; servo settings stay unchanged.

    As in LeRobot, the joints except wrist_roll are moved through their whole travel while a live
    table shows MIN, POS and MAX; each travel must include the middle (the model zero pose that
    center sets). wrist_roll turns fully and keeps the whole circle.
    """
    ids = list(arm_ids) + ([gripper_id] if gripper_id is not None else [])
    check_servos(bus, ids)
    offsets = {motor_id: offset(bus, motor_id) for motor_id in ids}

    def hardware_limits(motor_id):
        minimum, maximum = bus.read(motor_id, 9, 2), bus.read(motor_id, 11, 2)
        if not 0 <= minimum < maximum <= 4095:
            raise ValueError(f"servo {motor_id}: invalid hardware limits {minimum}..{maximum}")
        return minimum, maximum

    def sample(motor_id, message):
        prompt(message + " Press Enter to capture, Ctrl+C to cancel: ")
        check_servos(bus, ids, offsets)
        value = stationary(bus, motor_id)
        minimum, maximum = hardware_limits(motor_id)
        if not minimum <= value <= maximum:
            raise ValueError(f"servo {motor_id}: position outside configured hardware limits")
        return value

    joints = {name: motor_id for name, motor_id in zip(JOINTS, arm_ids) if name != FULL_TURN}
    arm = {}
    if joints:
        print(
            f"Support the arm and move {', '.join(joints)} one at a time, slowly, through the "
            f"whole travel to both ends.\n{FULL_TURN} turns fully and is not recorded. Press "
            "Enter when every joint has been moved, Ctrl+C to cancel."
        )
        with unechoed_input():
            minimums, maximums, wrapped = sweep(bus, joints, enter_pressed, show, period_s)
        check_servos(bus, ids, offsets)
        problems = []
        for name, motor_id in joints.items():
            low, high = minimums[name], maximums[name]
            minimum, maximum = hardware_limits(motor_id)
            if name in wrapped:
                problems.append(f"{name}: the reading wrapped around the encoder")
            elif low < minimum or high > maximum:
                problems.append(f"{name}: position outside configured hardware limits")
            elif high - low < 32:
                problems.append(f"{name}: travel {low}..{high} is too small (minimum 32 ticks)")
            elif not low <= MIDDLE <= high:
                problems.append(
                    f"{name}: travel {low}..{high} does not include the middle {MIDDLE}"
                )
        if problems:
            raise ValueError(
                "; ".join(problems) + ". Move each joint through its whole travel and capture "
                "again; run `so101-calibrate center` first if the middle is missing"
            )
        for name, motor_id in joints.items():
            arm[name] = {
                "id": motor_id,
                "homing_offset": offsets[motor_id],
                "range_min": minimums[name],
                "range_max": maximums[name],
            }
    for name, motor_id in zip(JOINTS, arm_ids):
        if name == FULL_TURN:
            arm[name] = {
                "id": motor_id,
                "homing_offset": offsets[motor_id],
                "range_min": 0,
                "range_max": 4095,
            }
    arm = {name: arm[name] for name in JOINTS if name in arm}
    gripper = None
    if gripper_id is not None:
        opened = sample(
            gripper_id, "Move the empty gripper to its open endpoint without forcing it."
        )
        closed = sample(
            gripper_id, "Move the empty gripper to its closed endpoint without forcing it."
        )
        if abs(opened - closed) < 100:
            raise ValueError("gripper travel is too small (minimum 100 encoder ticks)")
        gripper = {
            "motor_id": gripper_id,
            "open_ticks": opened,
            "closed_ticks": closed,
            "homing_offset": offsets[gripper_id],
        }
    check_servos(bus, ids, offsets)
    return arm, gripper


def save_new(path, value):
    """Preserve existing calibrations, including symlinks, by exclusive creation."""
    path = Path(path).absolute()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"saved {path}")


def default_site():
    """As the manipulation command: MANIPULATION_SITE, otherwise the user data directory."""
    if os.environ.get("MANIPULATION_SITE"):
        return os.environ["MANIPULATION_SITE"]
    data = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local/share")
    return str(Path(data) / "spacemit-manipulation/site")


def load_calibration(path):
    if not path.is_file():
        raise FileNotFoundError(
            f"{path} not found; run `so101-calibrate capture` first, or choose --part"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "operation",
        choices=("all", "status", "configure", "center", "capture", "limits"),
        help="all runs configure, center, capture and limits in order; status reads registers; "
        "configure writes recommended parameters; center sets the homing offsets; capture "
        "records travel to new calibration files; limits installs the captured travel as "
        "servo limits",
    )
    parser.add_argument(
        "--port", required=True, help="SO101 serial device, preferably /dev/serial/by-id/..."
    )
    parser.add_argument(
        "--site",
        default=default_site(),
        help="site directory; the calibration files are in its so101/ subdirectory "
        "(default: $MANIPULATION_SITE or ~/.local/share/spacemit-manipulation/site)",
    )
    parser.add_argument(
        "--part",
        choices=("all", "arm", "gripper"),
        default="all",
        help="servos to use (default all)",
    )
    parser.add_argument(
        "--arm-calibration",
        default=ARM_FILE,
        help=f"arm calibration file, relative to SITE/so101 (default {ARM_FILE})",
    )
    parser.add_argument(
        "--gripper-calibration",
        default=GRIPPER_FILE,
        help=f"gripper calibration file, relative to SITE/so101 (default {GRIPPER_FILE})",
    )
    parser.add_argument("--arm-ids", default="1,2,3,4,5")
    parser.add_argument("--gripper-id", type=int, default=6)
    parser.add_argument("--library", help="explicit libmanipulation_feetech_bus.so path")
    args = parser.parse_args()
    site = Path(args.site).expanduser().absolute() / "so101"
    files = dict(arm=site / args.arm_calibration, gripper=site / args.gripper_calibration)
    parts = [part for part in ("arm", "gripper") if args.part in ("all", part)]
    arm_ids = [int(item) for item in args.arm_ids.split(",")]
    if len(arm_ids) != len(JOINTS):
        raise ValueError("--arm-ids requires five IDs in joint order")
    if args.operation == "limits":
        arm = load_calibration(files["arm"]) if "arm" in parts else None
        gripper = load_calibration(files["gripper"]) if "gripper" in parts else None
        arm_ids = [arm[name]["id"] for name in JOINTS] if arm else []
        gripper_id = gripper["motor_id"] if gripper else None
    else:
        if args.operation in ("all", "capture"):
            if "arm" in parts and "gripper" in parts and files["arm"] == files["gripper"]:
                raise ValueError("arm and gripper calibration files must differ")
            for part in parts:
                if os.path.lexists(files[part]):
                    raise FileExistsError(
                        f"existing calibration kept: {files[part]}; move it aside to capture again"
                    )
        arm_ids = arm_ids if "arm" in parts else []
        gripper_id = args.gripper_id if "gripper" in parts else None
    ids = arm_ids + ([gripper_id] if gripper_id is not None else [])
    if len(set(ids)) != len(ids) or any(not 1 <= value <= 253 for value in ids):
        raise ValueError("servo IDs must be distinct integers in 1..253")
    if args.operation == "capture":
        print(
            "Travel capture preserves the current homing offsets: run `so101-calibrate center`\n"
            "first on a new or reassembled arm. Support the arm, remove loads, and release torque\n"
            "using the device procedure before starting. This operation only reads servo registers."
        )
    elif args.operation == "all":
        print(
            "Full setup: parameters, mid position, travel and servo limits. Support the arm,\n"
            "remove loads, and release torque before starting; this tool never enables torque."
        )
    elif args.operation != "status":
        print(
            "Torque must already be released with the arm supported; "
            "this tool never enables torque."
        )

    def save(arm, gripper):
        if arm_ids:
            save_new(files["arm"], arm)
        if gripper_id is not None:
            save_new(files["gripper"], gripper)

    bus = RegisterBus(library_path(args.library), args.port, ids)
    try:
        if args.operation == "all":
            calibrate_all(bus, arm_ids, gripper_id, save)
            print("Calibration complete: parameters, mid position, travel and servo limits.")
        elif args.operation == "capture":
            save(*capture(bus, arm_ids, gripper_id))
        elif args.operation == "status":
            for motor_id, values in status(bus, ids).items():
                print(json.dumps(dict(id=motor_id, **values)))
        elif args.operation == "configure":
            for motor_id, written in configure(bus, arm_ids, gripper_id).items():
                print(
                    f"servo {motor_id}: "
                    + (
                        ", ".join(f"register {address} = {value}" for address, _, value in written)
                        or "unchanged"
                    )
                )
        elif args.operation == "center":
            for motor_id, value in center(bus, ids).items():
                print(f"servo {motor_id}: homing offset {value}, limits 0..4095")
            print(
                "Existing calibration files no longer match: "
                "move them aside and capture travel again."
            )
        else:
            for motor_id, written in limits(bus, arm, gripper).items():
                print(f"servo {motor_id}: " + ("limits written" if written else "limits unchanged"))
    finally:
        bus.close()


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, KeyboardInterrupt, EOFError) as error:
        print(f"so101-calibrate: {error}", file=sys.stderr)
        raise SystemExit(1)
