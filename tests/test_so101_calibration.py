#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Travel capture, register status, parameter, mid-position and limit writes, and preservation of
existing site calibration files."""

import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "drivers/arms/so101/tools/calibrate.py"
SPEC = importlib.util.spec_from_file_location("so101_calibration", SOURCE)
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


def moves(target, *frames):
    """enter_pressed for a travel sweep: each call applies the next frame of positions (a dict of
    channel to position); Enter is pressed once every frame has been read."""
    frames = iter(frames)

    def pressed():
        frame = next(frames, None)
        if frame is None:
            return True
        target.update(frame)
        return False

    return pressed


def endpoints(target, *steps):
    """A prompt that, per call, sets one channel position (None: nothing moves)."""
    steps = iter(steps)

    def prompt(_):
        step = next(steps)
        if step is not None:
            target[step[0]] = step[1]

    return prompt


def quiet(text, final=False):
    """No live table in tests."""


class Bus:
    def __init__(self):
        self.positions = {}
        self.torque = 0
        self.homing = 0x0801
        self.model = 777
        self.error = 0

    def read(self, motor_id, address, length):
        return {
            3: self.model,
            9: 100,
            11: 4000,
            31: self.homing,
            33: 0,
            40: self.torque,
            56: self.positions.get(motor_id, 2048),
            65: self.error,
        }[address]


class Servos:
    """STS3215 register memory behind the bus: Present_Position is the actual position minus the
    homing offset. The motor driver, not the tool, handles the EEPROM lock."""

    def __init__(self, ids, actual=2048):
        self.memory, self.actual, self.writes, self.fail_at = {}, {}, [], None
        for motor_id in ids:
            memory = bytearray(71)
            for address, length, value in (
                (3, 2, 777),
                (7, 1, 250),
                (11, 2, 4095),
                (16, 2, 1000),
                (21, 1, 32),
                (28, 2, 500),
                (36, 1, 20),
                (41, 1, 0),
                (calibration.LOCK, 1, 1),
            ):
                memory[address : address + length] = value.to_bytes(length, "little")
            self.memory[motor_id], self.actual[motor_id] = memory, actual

    def set(self, motor_id, address, length, value):
        self.memory[motor_id][address : address + length] = value.to_bytes(length, "little")

    def registers(self, motor_id, address, length):
        memory = self.memory[motor_id]
        homing = calibration.decode_offset(int.from_bytes(memory[31:33], "little"))
        memory[56:58] = ((self.actual[motor_id] - homing) % 4096).to_bytes(2, "little")
        return bytes(memory[address : address + length])

    def read(self, motor_id, address, length):
        return int.from_bytes(self.registers(motor_id, address, length), "little")

    def write(self, motor_id, address, length, value):
        if self.fail_at == len(self.writes):
            self.fail_at = None
            raise OSError(5, "transport failure")
        self.writes.append((motor_id, address, value))
        if address in (40, calibration.LOCK):
            raise AssertionError(f"register {address} must never be written by the tool")
        self.set(motor_id, address, length, value)


class RegisterTests(unittest.TestCase):
    def test_offset_encoding_round_trip(self):
        for value in (-2047, -1, 0, 1, 2047):
            self.assertEqual(calibration.decode_offset(calibration.encode_offset(value)), value)
        self.assertEqual(calibration.encode_offset(-1), 0x0801)
        with self.assertRaises(ValueError):
            calibration.encode_offset(2048)
        with self.assertRaises(ValueError):
            calibration.decode_offset(0x1000)

    def test_status_decodes_without_writes(self):
        servos = Servos([1])
        servos.set(1, 31, 2, 0x0805)
        servos.set(1, 60, 2, 0x0400 | 120)
        servos.set(1, 62, 1, 121)
        values = calibration.status(servos, [1])[1]
        self.assertEqual(
            (values["model"], values["homing_offset"], values["load"], values["voltage_v"]),
            (777, -5, -120, 12.1),
        )
        self.assertEqual((values["position"], values["eeprom_locked"], values["p"]), (2053, 1, 32))
        self.assertEqual(servos.writes, [])

    def test_configure_writes_only_differences(self):
        servos = Servos([1, 2, 3, 4, 5, 6])
        servos.set(3, 21, 1, 16)
        written = calibration.configure(servos, [1, 2, 3, 4, 5], 6)
        self.assertEqual(written[1], [(7, 1, 0), (21, 1, 16)])
        self.assertEqual(written[3], [(7, 1, 0)])
        self.assertEqual(
            written[6], [(7, 1, 0), (21, 1, 16), (16, 2, 500), (28, 2, 250), (36, 1, 25)]
        )
        for motor_id in range(1, 7):
            self.assertEqual(servos.read(motor_id, calibration.LOCK, 1), 1)
            self.assertEqual(servos.read(motor_id, 40, 1), 0)
        self.assertEqual(servos.read(6, 16, 2), 500)
        self.assertEqual(servos.read(1, 16, 2), 1000)  # Arm torque limit unchanged.
        count = len(servos.writes)
        self.assertEqual(
            calibration.configure(servos, [1, 2, 3, 4, 5], 6), {i: [] for i in range(1, 7)}
        )
        self.assertEqual(len(servos.writes), count)  # Equal values are not written again.

    def test_configure_sets_position_mode_and_refuses_torque_on(self):
        servos = Servos([1])
        servos.set(1, 33, 1, 3)
        self.assertIn((33, 1, 0), calibration.configure(servos, [1], None)[1])
        servos.set(1, 40, 1, 1)
        servos.set(1, 21, 1, 30)
        count = len(servos.writes)
        with self.assertRaisesRegex(ValueError, "torque must already be off"):
            calibration.configure(servos, [1], None)
        self.assertEqual(len(servos.writes), count)

    def test_failed_write_stops_the_servo_and_the_run(self):
        servos = Servos([1, 2])
        servos.fail_at = 1  # The first parameter is written, the second fails.
        with self.assertRaises(OSError):
            calibration.configure(servos, [1, 2], None)
        self.assertEqual(servos.writes, [(1, 7, 0)])
        self.assertEqual(servos.read(2, 7, 1), 250)

    def test_servo_error_is_refused_before_any_write(self):
        servos = Servos([1])
        servos.set(1, 65, 1, 4)
        with self.assertRaisesRegex(ValueError, "reports error 4"):
            calibration.configure(servos, [1], None)
        self.assertEqual(servos.writes, [])

    def test_center_makes_the_held_pose_the_middle(self):
        servos = Servos([1, 2], actual=1000)
        servos.actual[2] = 3500
        servos.set(1, 31, 2, calibration.encode_offset(-300))  # A previous calibration.
        servos.set(1, 9, 2, 700)
        servos.set(1, 11, 2, 1500)
        offsets = calibration.center(servos, [1, 2], lambda _: None, settle_s=0)
        self.assertEqual(offsets, {1: 1000 - 2047, 2: 3500 - 2047})
        for motor_id in (1, 2):
            self.assertEqual(servos.read(motor_id, 56, 2), calibration.MIDDLE)
            self.assertEqual((servos.read(motor_id, 9, 2), servos.read(motor_id, 11, 2)), (0, 4095))
            self.assertEqual(servos.read(motor_id, calibration.LOCK, 1), 1)

    def test_center_refuses_encoder_end_and_torque_on(self):
        with self.assertRaisesRegex(ValueError, "encoder end"):
            calibration.center(Servos([1], actual=4095), [1], lambda _: None, settle_s=0)
        servos = Servos([1])
        servos.set(1, 40, 1, 1)
        with self.assertRaisesRegex(ValueError, "torque must already be off"):
            calibration.center(
                servos, [1], lambda _: self.fail("must not prompt while torque is on")
            )

    def test_limits_install_captured_travel_in_its_offset_frame(self):
        servos = Servos([1, 2, 3, 4, 5, 6])
        arm = {
            name: {"id": index + 1, "homing_offset": 0, "range_min": 1000, "range_max": 3000}
            for index, name in enumerate(calibration.JOINTS)
        }
        gripper = {"motor_id": 6, "open_ticks": 2900, "closed_ticks": 1900, "homing_offset": 0}
        servos.set(4, 31, 2, calibration.encode_offset(10))
        with self.assertRaisesRegex(ValueError, "homing offset differs"):
            calibration.limits(servos, arm, gripper)
        servos.set(4, 31, 2, 0)
        servos.actual[6] = 3000
        with self.assertRaisesRegex(ValueError, "into its captured range"):
            calibration.limits(servos, arm, gripper)
        self.assertEqual(servos.writes, [])
        servos.actual[6] = 2500
        calibration.limits(servos, arm, gripper)
        self.assertEqual(
            [(servos.read(i, 9, 2), servos.read(i, 11, 2)) for i in (1, 6)],
            [(1000, 3000), (1900, 2900)],
        )
        self.assertTrue(all(servos.read(i, calibration.LOCK, 1) == 1 for i in range(1, 7)))

    def test_all_runs_the_full_setup_in_order_and_saves_before_limits(self):
        servos = Servos([1, 2, 3, 4, 5, 6], actual=2048)
        prompt = endpoints(servos.actual, None, (6, 1501), (6, 2801))  # center, then the gripper
        # Joints 1-4 are moved one at a time; wrist_roll (5) is not recorded.
        sweep = moves(
            servos.actual,
            *[
                frame
                for motor in range(1, 5)
                for frame in ({motor: 1001}, {motor: 3001}, {motor: 2048})
            ],
        )

        saved = []

        def save(arm, gripper):
            saved.append((arm, gripper))
            self.assertEqual(
                servos.read(1, 9, 2), 0
            )  # Limits are still open when the files are saved.

        arm, gripper = calibration.calibrate_all(
            servos,
            [1, 2, 3, 4, 5],
            6,
            save,
            prompt,
            settle_s=0,
            enter_pressed=sweep,
            show=quiet,
            period_s=0,
        )
        self.assertEqual(saved, [(arm, gripper)])
        self.assertEqual(servos.read(1, 21, 1), 16)  # configure
        self.assertEqual(calibration.offset(servos, 1), 1)  # center: 2048 - 2047
        self.assertEqual(
            arm["shoulder_pan"], {"id": 1, "homing_offset": 1, "range_min": 1000, "range_max": 3000}
        )
        self.assertEqual(  # The whole circle: wrist_roll can turn past half a turn.
            arm["wrist_roll"], {"id": 5, "homing_offset": 1, "range_min": 0, "range_max": 4095}
        )
        self.assertEqual(
            gripper, {"motor_id": 6, "open_ticks": 1500, "closed_ticks": 2800, "homing_offset": 1}
        )
        self.assertEqual(
            [(servos.read(i, 9, 2), servos.read(i, 11, 2)) for i in (1, 5, 6)],
            [(1000, 3000), (0, 4095), (1500, 2800)],
        )
        self.assertFalse(any(address == 40 for _, address, _ in servos.writes))

    def test_capture_writes_nothing_and_matches_limits(self):
        servos = Servos([6])
        prompt = endpoints(servos.actual, (6, 2900), (6, 1900))
        _, gripper = calibration.capture(servos, [], 6, prompt)
        self.assertEqual(servos.writes, [])
        calibration.limits(servos, None, json.loads(json.dumps(gripper)))
        self.assertEqual((servos.read(6, 9, 2), servos.read(6, 11, 2)), (1900, 2900))


class CaptureTests(unittest.TestCase):
    def test_capture_arm_and_reversed_gripper(self):
        bus = Bus()
        sweep = moves(
            bus.positions,
            {1: 1500, 3: 1000},
            {1: 2600, 3: 2600},
            {1: 3000, 2: 1000, 4: 1000},
            {2: 3000, 3: 3000, 4: 3000},
            {1: 2048, 2: 2048, 3: 2048, 4: 2048},
        )
        prompt = endpoints(bus.positions, (6, 1500), (6, 2800))
        arm, gripper = calibration.capture(bus, list(range(1, 6)), 6, prompt, sweep, quiet, 0)
        self.assertEqual(set(arm), set(calibration.JOINTS))
        self.assertEqual(  # Every position passed counts, not only where the joint stopped.
            arm["elbow_flex"], {"id": 3, "homing_offset": -1, "range_min": 1000, "range_max": 3000}
        )
        self.assertEqual(
            arm["wrist_roll"], {"id": 5, "homing_offset": -1, "range_min": 0, "range_max": 4095}
        )
        self.assertEqual(
            gripper, {"motor_id": 6, "open_ticks": 1500, "closed_ticks": 2800, "homing_offset": -1}
        )

    def test_enabled_device_is_refused_before_prompt(self):
        bus = Bus()
        bus.torque = 1
        with self.assertRaisesRegex(ValueError, "torque must already be off"):
            calibration.capture(
                bus,
                [1, 2, 3, 4, 5],
                None,
                lambda _: self.fail("must not prompt while torque is on"),
            )

    def test_changed_offset_is_refused(self):
        bus = Bus()

        def prompt(_):
            bus.homing = 0

        with self.assertRaisesRegex(ValueError, "offset changed"):
            calibration.capture(bus, [], 6, prompt)

    def test_small_range_is_refused(self):
        with self.assertRaisesRegex(ValueError, "travel is too small"):
            calibration.capture(Bus(), [], 6, lambda _: None)
        sweep = moves({}, {})  # No joint moves before Enter.
        with self.assertRaisesRegex(ValueError, "shoulder_pan: travel 2048..2048 is too small"):
            calibration.capture(Bus(), [1, 2, 3, 4, 5], None, lambda _: None, sweep, quiet, 0)

    def test_every_problem_is_reported_at_once(self):
        bus = Bus()
        bus.positions.update({1: 1000, 2: 1000, 3: 1000, 4: 1000})  # Not centered.
        sweep = moves(bus.positions, {1: 1500, 2: 3000, 3: 4090}, {3: 20, 4: 1001})
        with self.assertRaises(ValueError) as raised:
            calibration.capture(bus, [1, 2, 3, 4, 5], None, lambda _: None, sweep, quiet, 0)
        message = str(raised.exception)
        self.assertIn("shoulder_pan: travel 1000..1500 does not include the middle 2047", message)
        self.assertNotIn("shoulder_lift", message)  # 1000..3000 includes the middle.
        self.assertIn("elbow_flex: the reading wrapped around the encoder", message)
        self.assertIn("wrist_flex: travel 1000..1001 is too small", message)

    def test_cancel_stops_the_sweep(self):
        def pressed():
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            calibration.capture(Bus(), [1, 2, 3, 4, 5], None, lambda _: None, pressed, quiet, 0)

    def test_live_table_redraws_in_place_on_a_terminal_only(self):
        class Terminal(io.StringIO):
            def isatty(self):
                return True

        text = calibration.travel_table(
            ["shoulder_pan"], {"shoulder_pan": 1}, {"shoulder_pan": 2}, {"shoulder_pan": 3}
        )
        self.assertIn("shoulder_pan    |      1 |      2 |      3", text)
        terminal, log = Terminal(), io.StringIO()
        for stream in (terminal, log):
            table = calibration.LiveTable(stream)
            table(text)
            table(text)
            table(text, final=True)
        self.assertEqual(terminal.getvalue(), text + f"\033[{text.count(chr(10))}A" + text)
        self.assertEqual(log.getvalue(), text)  # Only the final table in a log.

    def test_hardware_boundary_is_enforced(self):
        bus = Bus()
        bus.positions[6] = 4095
        with self.assertRaisesRegex(ValueError, "outside configured hardware limits"):
            calibration.capture(bus, [], 6, lambda _: None)

    def test_existing_file_and_symlink_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = root / "original.json"
            original.write_text("original\n", encoding="utf-8")
            alias = root / "alias.json"
            alias.symlink_to(original)
            dangling = root / "dangling.json"
            missing = root / "missing.json"
            dangling.symlink_to(missing)
            for path in (original, alias, dangling):
                with self.assertRaises(FileExistsError):
                    calibration.save_new(path, {"new": True})
            self.assertEqual(original.read_text(), "original\n")
            self.assertFalse(missing.exists())


if __name__ == "__main__":
    unittest.main()
