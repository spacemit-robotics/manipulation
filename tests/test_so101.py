#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Registered SO101 driver through real PTY packets and the normal local device service."""
import errno
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402

CLI = os.environ["MANIPULATION_DEVICE"]
LIBRARY = os.environ["MANIPULATION_FEETECH_BUS"]
CALIBRATE = ROOT / "drivers/arms/so101/tools/calibrate.py"
SPEC = importlib.util.spec_from_file_location("so101_calibration", CALIBRATE)
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class ServoBus:
    """Six register maps with no external devices, deterministic reads and acknowledged writes."""

    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.registers = {motor_id: bytearray(256) for motor_id in range(1, 7)}
        for motor_id in self.registers:
            self.word(motor_id, 3, 777)
            self.registers[motor_id][0:2] = b"\x03\x0a"
            self.word(motor_id, 9, 100)
            self.word(motor_id, 11, 3995)
            self.word(motor_id, 56, 2048 if motor_id < 6 else 2750)
            self.registers[motor_id][40] = 1
        self.requests = []
        self.writes = []
        self.failure = None
        self.drop_goals = False
        self.stall = {}  # motor ID -> encoder position where an object blocks a goal
        self.response_delay = 0
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def word(self, motor_id, address, value):
        self.registers[motor_id][address:address + 2] = value.to_bytes(2, "little")

    def close(self):
        self.done.set()
        self.thread.join(1)
        os.close(self.master)
        os.close(self.slave)

    def run(self):
        pending = bytearray()
        while not self.done.is_set():
            if not select.select([self.master], [], [], 0.02)[0]:
                continue
            try:
                pending.extend(os.read(self.master, 1024))
            except OSError:
                return
            while len(pending) >= 6 and len(pending) >= pending[3] + 4:
                count = pending[3] + 4
                packet = bytes(pending[:count])
                del pending[:count]
                if packet[:2] != b"\xff\xff" or sum(packet[2:]) & 255 != 255:
                    return
                motor_id, instruction = packet[2], packet[4]
                parameters = packet[5:-1]
                self.requests.append((motor_id, instruction, parameters))
                if instruction == 3:
                    address, data = parameters[0], parameters[1:]
                    self.writes.append((motor_id, address, data))
                    if not (address == 42 and self.drop_goals):  # Acknowledged even when lost.
                        self.registers[motor_id][address:address + len(data)] = data
                        if address == 42 and self.registers[motor_id][40]:
                            stop = self.stall.get(motor_id)
                            self.registers[motor_id][56:58] = data[:2] if stop is None else stop.to_bytes(2, "little")
                    payload = b""
                elif instruction == 2:
                    address, width = parameters
                    payload = bytes(self.registers[motor_id][address:address + width])
                else:
                    return
                body = bytes([motor_id, len(payload) + 2, 0]) + payload
                reply = b"\xff\xff" + body + bytes([(~sum(body)) & 255])
                failure = self.failure
                if failure == "timeout":
                    continue
                if failure == "checksum":
                    reply = reply[:-1] + bytes([reply[-1] ^ 1])
                if failure == "partial":
                    reply = reply[:-10]
                if failure == "id":
                    reply = reply[:2] + bytes([motor_id + 1]) + reply[3:]
                try:
                    if self.response_delay:
                        time.sleep(self.response_delay)
                    if failure == "fragment":
                        for offset in range(0, len(reply), 7):
                            os.write(self.master, reply[offset:offset + 7])
                            time.sleep(0.0005)
                    else:
                        os.write(self.master, reply)
                except OSError:
                    return


class So101Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="so101-contract-")
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.bus = ServoBus()
        self.addCleanup(self.bus.close)
        self.arm_calibration = {name: dict(id=i + 1, homing_offset=0,
                                         range_min=300, range_max=3800) for i, name in enumerate(JOINTS)}
        self.gripper_calibration = dict(motor_id=6, closed_ticks=2031, open_ticks=3468)
        self.plan = dict(schema_version=3, device=dict(driver="so101", model="so101",
            arm=dict(device_id="arm_main", resource_id="so101_test_" + self.directory.name,
                     maximum_age_s=0.5, joints=[dict(name=name, minimum_rad=-2, maximum_rad=2)
                                               for name in JOINTS]),
            options=dict(port=self.bus.port, arm_calibration="arm.json")), steps=[])
        self.filename = self.directory / "device.json"
        self.save()

    def save(self):
        self.filename.write_text(json.dumps(self.plan))
        (self.directory / "arm.json").write_text(json.dumps(self.arm_calibration))
        (self.directory / "gripper.json").write_text(json.dumps(self.gripper_calibration))

    def gripper(self):
        self.plan["device"]["effector"] = dict(device_id="gripper_main", channels=["gripper"],
                                             feedback="position", maximum_age_s=0.5)
        self.plan["device"]["options"]["gripper_calibration"] = "gripper.json"
        self.save()

    def client(self, motion=True):
        return DeviceClient(CLI, self.filename, allow_motion=motion, allow_enable=motion,
                            allow_disable=motion, allow_effector_enable=motion,
                            allow_effector_disable=motion)

    def call(self, command):
        return subprocess.run([CLI, command, str(self.filename)], text=True, capture_output=True, timeout=5)

    def test_connect_inspect_disconnect_never_writes(self):
        self.gripper()
        with self.client(False) as client:
            state = client.state()
            self.assertEqual(len(state["joints_rad"]), 5)
            self.assertAlmostEqual(state["joints_rad"][0], 2048 * 2 * math.pi / 4095 - math.pi)
            self.assertAlmostEqual(state["effector_positions"][0], (2750 - 2031) / 1437)
            self.assertTrue(state["effector_enabled"])
        self.assertTrue(self.bus.requests)
        self.assertTrue(all(instruction == 2 for _, instruction, _ in self.bus.requests))
        self.assertEqual(self.bus.writes, [])

    def test_missing_arm_calibration_allows_inspect_but_rejects_enable_and_move(self):
        del self.plan["device"]["options"]["arm_calibration"]
        self.save()
        with self.client() as client:
            self.assertEqual(len(client.state()["joints_rad"]), 5)
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1] * 5, 0.1)
            with self.assertRaises(DeviceCommandError):
                client.enable_arm()
        self.assertEqual(self.bus.writes, [])

    def test_config_validation_is_offline_and_rejects_unusable_mappings(self):
        self.assertEqual(self.call("validate").returncode, 0)
        self.assertEqual(self.bus.requests, [])
        self.plan["device"]["options"]["arm_ids"] = "1,1,3,4,5"
        self.save()
        self.assertNotEqual(self.call("validate").returncode, 0)
        self.assertEqual(self.bus.requests, [])

    def test_joint_point_mapping_holding_and_explicit_torque(self):
        with self.client() as client:
            result = client.move_arm([0.1, -0.2, 0.3, -0.4, 0.5], 0.1)
            self.assertEqual(result["status"], "succeeded")
            for index, target in enumerate([0.1, -0.2, 0.3, -0.4, 0.5]):
                ticks = int.from_bytes(self.bus.registers[index + 1][56:58], "little")
                self.assertEqual(ticks, round((target + math.pi) * 4095 / (2 * math.pi)))
            self.assertEqual(client.hold_arm()["status"], "succeeded")
            self.assertTrue(all(self.bus.registers[i][40] == 1 for i in range(1, 6)))
            client.disable_arm()
            self.assertFalse(client.state()["enabled"])
            writes_before = len(self.bus.writes)
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.0] * 5, 0.1)
            self.assertEqual(writes_before, len(self.bus.writes))
            client.enable_arm()
            self.assertTrue(client.state()["enabled"])
            new_writes = self.bus.writes[writes_before:]
            self.assertEqual([address for _, address, _ in new_writes], [42] * 5 + [40] * 5)

    def test_target_outside_calibration_is_rejected_without_writes(self):
        for name in JOINTS:
            self.arm_calibration[name].update(range_min=1800, range_max=2300)
        self.save()
        with self.client() as client:
            with self.assertRaises(DeviceCommandError):
                client.move_arm([1] * 5, 0.1)
        self.assertEqual(self.bus.writes, [])

    def test_enable_requires_goal_readback_before_any_torque_write(self):
        self.gripper()
        for motor_id in range(1, 7):
            self.bus.registers[motor_id][40] = 0
            self.bus.word(motor_id, 42, 3100)
        self.bus.drop_goals = True
        for effector in (False, True):
            with self.subTest(effector=effector), self.client() as client:
                with self.assertRaises(DeviceCommandError):
                    client.enable_effector() if effector else client.enable_arm()
                self.assertTrue(all(self.bus.registers[i][40] == 0 for i in range(1, 7)))
        self.assertFalse(any(address == 40 for _, address, _ in self.bus.writes))

    def test_encoder_speed_is_bounded_and_unrepresentable_speed_rejected(self):
        with self.client() as client:
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1] * 5, 0.00001)
            self.assertEqual(self.bus.writes, [])
            client.move_arm([0.1] * 5, 0.123)
        speed_tick_rad_s = 2 * math.pi / 4096
        speeds = [int.from_bytes(data[4:6], "little") for _, address, data in self.bus.writes if address == 42]
        self.assertTrue(speeds)
        self.assertTrue(all(0 < value * speed_tick_rad_s <= 0.123 for value in speeds))

    def test_wrong_model_rejected_before_any_write(self):
        self.bus.word(3, 3, 888)
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("model 777", result.stderr)
        self.assertEqual(self.bus.writes, [])

    def test_complete_sample_age_checked_before_writes(self):
        self.plan["device"]["arm"]["maximum_age_s"] = 0.05
        self.save()
        with self.client() as client:
            self.bus.response_delay = 0.015
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1] * 5, 0.1)
        self.assertEqual(self.bus.writes, [])

    def test_gripper_offset_evidence_and_session_pin(self):
        self.gripper()
        self.gripper_calibration["homing_offset"] = 10
        self.save()
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("homing offset", result.stderr)
        del self.gripper_calibration["homing_offset"]
        self.save()
        with self.client(False) as client:
            self.bus.word(6, 31, 10)
            with self.assertRaises(DeviceCommandError):
                client.state()
        self.assertEqual(self.bus.writes, [])

    def test_calibration_offset_mismatch_rejects_connection_without_writes(self):
        self.bus.word(2, 31, 7)
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("calibration differs", result.stderr)
        self.assertEqual(self.bus.writes, [])

    def test_gripper_both_directions_and_explicit_torque(self):
        self.gripper()
        for reverse in (False, True):
            if reverse:
                self.gripper_calibration.update(open_ticks=2031, closed_ticks=3468)
                self.save()
            with self.client() as client:
                for position in (0, 0.5, 1):
                    self.assertEqual(client.move_effector([position])["status"], "succeeded")
                    self.assertAlmostEqual(client.state()["effector_positions"][0], position, delta=0.001)
                self.assertEqual(client.hold_effector()["status"], "succeeded")
                client.disable_effector()
                self.assertFalse(client.state()["effector_enabled"])
                client.enable_effector()
                self.assertTrue(client.state()["effector_enabled"])

    def test_follow_replaces_targets_while_moving_and_stops_on_hold(self):
        self.bus.stall = {motor_id: 2048 for motor_id in range(1, 6)}  # The arm never reaches a target here.
        with self.client() as client:
            goals = ([0.05] * 5, [0.1] * 5, [0.15] * 5)
            result = client.follow_arm(goals[0], 0.5)
            self.assertEqual((result["status"], result["detail"][:4]), ("succeeded", "sent"))
            self.bus.registers[1][66] = 1  # Now reported moving: a later target may replace the goal.
            for goal in goals[1:]:
                self.assertEqual(client.follow_arm(goal, 0.5)["status"], "succeeded")
            sent = [data for motor_id, address, data in self.bus.writes if motor_id == 1 and address == 42]
            self.assertEqual(len(sent), 3)
            self.assertEqual(int.from_bytes(sent[-1][:2], "little"), round((0.15 + math.pi) * 4095 / (2 * math.pi)))
            self.bus.registers[1][66] = 0
            self.assertEqual(client.hold_arm()["status"], "succeeded")
            goal = bytes(self.bus.registers[1][42:44])
            self.assertEqual(int.from_bytes(goal, "little"), 2048)  # Hold stops where the arm is.
            self.assertEqual(client.state()["fault"], False)

    def test_point_moves_hand_over_while_the_arm_still_moves(self):
        # line-move on a driver without paths: the next waypoint starts before the arm is still.
        with self.client() as client:
            self.assertIn("within_tolerance", client.move_arm([0.05] * 5, 0.5, wait_for_arrival=False)["detail"])
            self.bus.registers[1][66] = 1
            self.assertIn("within_tolerance", client.move_arm([0.1] * 5, 0.5, wait_for_arrival=False)["detail"])
            self.bus.registers[1][66] = 0
            self.assertIn("target_reached", client.move_arm([0.15] * 5, 0.5)["detail"])

    def test_follow_needs_a_stationary_start_and_valid_target_before_any_write(self):
        self.bus.registers[1][66] = 1
        with self.client() as client:
            with self.assertRaises(DeviceCommandError):
                client.follow_arm([0.05] * 5, 0.5)
            self.bus.registers[1][66] = 0
            with self.assertRaises(DeviceCommandError):
                client.follow_arm([3.0] * 5, 0.5)
            self.bus.registers[1][40] = 0
            with self.assertRaises(DeviceCommandError):
                client.follow_arm([0.05] * 5, 0.5)
        self.assertEqual(self.bus.writes, [])

    def test_lost_follow_target_latches_the_session(self):
        self.bus.drop_goals = True
        with self.client() as client:
            with self.assertRaises(DeviceCommandError) as lost:
                client.follow_arm([0.05] * 5, 0.5)
            self.assertIn(lost.exception.result["status"], ("fault", "stop_unknown"))
            with self.assertRaises(DeviceCommandError):
                client.follow_arm([0.05] * 5, 0.5)

    def test_gripper_follow_reports_grasp_evidence_without_waiting(self):
        self.gripper()
        self.bus.stall[6] = 2600
        self.bus.word(6, 60, 0x400 | 300)
        with self.client() as client:
            result = client.follow_effector([0.0])
            self.assertEqual((result["status"], result["detail"][:4]), ("succeeded", "sent"))
            state = client.state()
            self.assertTrue(state["effector_holding"])
            self.assertAlmostEqual(state["effector_loads"][0], -0.3)

    def test_gripper_grasp_holding_on_load_and_empty_close(self):
        self.gripper()
        with self.client() as client:
            self.assertTrue(client.capabilities()["effector"]["grasp_detection"])
            # Closing on an object: blocked short of the closed position, load above grip_load_threshold.
            self.bus.stall[6] = 2600
            self.bus.word(6, 60, 0x400 | 300)  # Present_Load 300 in the negative direction (bit 10).
            result = client.move_effector([0])
            self.assertEqual((result["status"], result["detail"], result["physical_grip_verified"]),
                             ("succeeded", "effector_grasp_holding", True))
            state = client.state()
            self.assertTrue(state["effector_holding"])
            self.assertAlmostEqual(state["effector_loads"][0], -0.3)
            goals = [write for write in self.bus.writes if write[:2] == (6, 42)]
            self.assertEqual(client.hold_effector()["status"], "succeeded")
            self.assertEqual([write for write in self.bus.writes if write[:2] == (6, 42)], goals,
                             "a hold released the grasp")
            # Holding long enough trips overload protection (status bit 5): still a grasp, not a fault.
            self.bus.registers[6][65] = 0x20
            self.assertFalse(client.state()["effector_fault"])
            self.bus.registers[6][65] = 0
            # Nothing in the hand: closing reaches the closed position, which is not a grasp.
            del self.bus.stall[6]
            self.bus.word(6, 60, 0)
            self.assertEqual(client.move_effector([1])["detail"], "effector_position_reached")
            result = client.move_effector([0])
            self.assertEqual((result["detail"], result["physical_grip_verified"]), ("effector_position_reached", False))
            self.assertFalse(client.state()["effector_holding"])
            self.bus.registers[6][65] = 0x04  # Any other servo error remains a fault.
            self.assertTrue(client.state()["effector_fault"])

    def test_gripper_load_below_threshold_is_not_a_grasp(self):
        self.gripper()
        self.plan["device"]["options"]["grip_load_threshold"] = 400
        self.plan["device"]["effector"]["timeout_s"] = 0.5
        self.save()
        self.bus.stall[6] = 2600
        self.bus.word(6, 60, 300)
        with self.client() as client:
            with self.assertRaises(DeviceCommandError) as raised:
                client.move_effector([0])
            self.assertIn(raised.exception.result["status"], ("timed_out", "stop_unknown"))

    def test_unconfigured_gripper_protection_refuses_enable_without_writes(self):
        self.gripper()
        self.bus.registers[6][40] = 0
        self.bus.word(6, 16, 1000)  # Factory Max_Torque_Limit: protection was never configured.
        with self.client() as client:
            with self.assertRaises(DeviceCommandError) as raised:
                client.enable_effector()
            self.assertIn("so101-calibrate configure", str(raised.exception))
        self.assertEqual([write for write in self.bus.writes if write[0] == 6], [])

    def test_gripper_missing_calibration_rejected_before_open(self):
        self.gripper()
        del self.plan["device"]["options"]["gripper_calibration"]
        self.save()
        self.assertNotEqual(self.call("inspect").returncode, 0)
        self.assertEqual(self.bus.requests, [])

    def test_faults_and_mode_are_reported_and_prevent_targets(self):
        for address in (33, 65):
            with self.subTest(address=address):
                self.bus.registers[3][address] = 1
                with self.client() as client:
                    with self.assertRaises(DeviceCommandError):
                        client.move_arm([0.1] * 5, 0.1)
                self.bus.registers[3][address] = 0
        self.assertEqual(self.bus.writes, [])

    def test_fragmented_feedback_is_accepted(self):
        self.bus.failure = "fragment"
        self.assertEqual(self.call("inspect").returncode, 0)
        self.assertEqual(self.bus.writes, [])

    def test_transport_errors_fail_bounded_without_writes(self):
        for failure in ("timeout", "partial", "checksum", "id"):
            with self.subTest(failure=failure):
                self.bus.failure = failure
                start = time.monotonic()
                result = self.call("inspect")
                self.assertNotEqual(result.returncode, 0)
                self.assertLess(time.monotonic() - start, 5)  # A stalled frame: about 3 s of driver retries.
        self.assertEqual(self.bus.writes, [])

    def test_port_alias_exclusive_even_with_distinct_resource_id(self):
        with self.client(False):
            alias = self.directory / "serial"
            alias.symlink_to(self.bus.port)
            self.plan["device"]["options"]["port"] = "serial"
            self.plan["device"]["arm"]["resource_id"] += "_other"
            self.save()
            result = self.call("inspect")
            self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.bus.writes, [])


class BusTests(unittest.TestCase):
    """The shared library over the motor driver, as so101-calibrate loads it."""

    def setUp(self):
        self.bus = ServoBus()
        self.addCleanup(self.bus.close)
        for motor_id in self.bus.registers:
            self.bus.registers[motor_id][40] = 0
            self.bus.registers[motor_id][21] = 16

    def tool(self, *arguments):
        return subprocess.run([sys.executable, str(CALIBRATE), *arguments, "--port", self.bus.port, "--library", LIBRARY],
                              text=True, capture_output=True, timeout=20)

    def test_status_keeps_stdout_machine_readable_and_never_writes(self):
        result = self.tool("status")
        self.assertEqual(result.returncode, 0, result.stderr)
        servos = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([servo["id"] for servo in servos], list(range(1, 7)))
        self.assertEqual(self.bus.writes, [])

    def test_configure_uses_the_motor_eeprom_lock_and_repeats_without_writes(self):
        self.bus.registers[1][21] = 32
        result = self.tool("configure", "--part", "arm")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.bus.registers[1][21], 16)
        written = [(address, bytes(data)) for motor_id, address, data in self.bus.writes if motor_id == 1]
        self.assertEqual(written, [(55, b"\x00"), (21, b"\x10"), (55, b"\x01")])
        self.assertFalse(any(address == 40 for _, address, _ in self.bus.writes))
        self.bus.writes.clear()
        self.assertEqual(self.tool("configure", "--part", "arm").returncode, 0)
        self.assertEqual(self.bus.writes, [])

    def test_lost_write_is_reported_though_the_servo_acknowledged_it(self):
        self.bus.drop_goals = True
        bus = calibration.RegisterBus(LIBRARY, self.bus.port, [1])
        self.addCleanup(bus.close)
        with self.assertRaisesRegex(OSError, "write not confirmed"):
            bus.write(1, 42, 2, 100)

    def test_second_owner_is_refused_until_the_first_releases(self):
        first = calibration.RegisterBus(LIBRARY, self.bus.port, [1])
        self.assertEqual(first.read(1, 3, 2), 777)
        with self.assertRaises(OSError) as refused:
            calibration.RegisterBus(LIBRARY, self.bus.port, [1])
        self.assertEqual(refused.exception.errno, errno.EBUSY)
        first.close()
        second = calibration.RegisterBus(LIBRARY, self.bus.port, [1])
        self.addCleanup(second.close)
        self.assertEqual(second.read(1, 3, 2), 777)


if __name__ == "__main__":
    unittest.main()
