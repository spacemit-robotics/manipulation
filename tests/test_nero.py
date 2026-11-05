#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Real C++/worker transport for the Nero driver against an explicitly selected, CAN-free SDK
fixture."""

import fcntl
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as Node
import unittest

ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "drivers/common/agx_session/agx_worker.py"
SPEC = importlib.util.spec_from_file_location("agx_worker", WORKER)
worker = importlib.util.module_from_spec(SPEC)
sys.dont_write_bytecode = True
SPEC.loader.exec_module(worker)
CLI = os.environ.get("MANIPULATION_DEVICE", "")
RECOVERY = os.environ.get("MANIPULATION_RECOVERY", "")
sys.path.insert(0, str(ROOT / "bindings/python"))
# Imported after the source binding path is added.
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402

CODES = {
    "can_fixture": "ok",
    "can_writefail": "write",
    "can_motorfault": "motor",
    "can_armfault": "fault",
    "can_badmode": "mode",
    "can_partial": "part",
    "can_stale": "stale",
    "can_wrongid": "id",
    "can_wrongv": "ver",
    "can_timeout": "time",
    "can_eof": "eof",
    "can_oversized": "large",
    "can_half": "half",
    "can_disabled": "disabled",
    "can_stall": "stall",
    "can_drop": "drop",
    "can_path": "path",
    "can_offset": "offset",
    "can_notreached": "nreach",
    "can_lag": "lag",
    "recover": "recover",
}


class NeroIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="nero-adapter-")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.sdk = self.directory / "sdk"
        shutil.copytree(ROOT / "tests/fake_sdk", self.sdk)
        (self.sdk / "UPSTREAM_COMMIT").write_text(worker.SDK_COMMIT + "\n", encoding="utf-8")
        self.plan = {
            "schema_version": 3,
            "device": {
                "driver": "agx_nero_revo2",
                "arm": {
                    "device_id": "arm_main",
                    "resource_id": "agx_can_fixture",
                    "joints": [
                        {"name": f"joint{i}", "minimum_rad": -1, "maximum_rad": 1}
                        for i in range(1, 8)
                    ],
                },
                "options": {
                    "channel": "can_fixture",
                    "worker": str(WORKER),
                    "sdk_root": str(self.sdk),
                    "python": os.path.realpath(sys.executable),
                },
            },
            "steps": [{"capability": "joint_point", "target_rad": [0.05] * 7, "speed_scale": 0.05}],
        }
        self.channel("can_fixture")

    @property
    def options(self):
        return self.plan["device"]["options"]

    @property
    def filename(self):
        path = self.directory / "plan.yaml"
        path.write_text(json.dumps(self.plan), encoding="utf-8")
        return path

    def call(self, command, *arguments, cwd=None):
        return subprocess.run(
            [CLI, command, str(self.filename), *arguments],
            capture_output=True,
            text=True,
            timeout=15,
            cwd=cwd,
        )

    def channel(self, name):
        # CAN names are at most 15 characters; the fixture reads its mode after the last '_'.
        channel = f"t{os.getpid() % 0x10000:04x}_{CODES[name]}"
        self.options["channel"] = channel
        self.plan["device"]["arm"]["resource_id"] = f"agx_{channel}"

    def events(self):
        trace = self.sdk / "pyAgxArm/recovery-events.jsonl"
        if not trace.exists():
            return []
        return [json.loads(line)["op"] for line in trace.read_text(encoding="utf-8").splitlines()]

    def test_release_registration(self):
        result = subprocess.run([CLI, "drivers"], capture_output=True, text=True, check=True)
        expected = {"mock_device", "agx_nero_revo2"}
        if os.environ.get("MANIPULATION_TEST_SO101") == "1":
            expected.add("so101")
        self.assertEqual(set(result.stdout.split()), expected)

    def test_validation_never_imports_sdk_or_spawns(self):
        self.options.update(sdk_root="/missing/sdk", python="/missing/python")
        self.assertEqual(self.call("validate").returncode, 0)

    @unittest.skipUnless(Path("/sys/class/net/lo").exists(), "Linux sysfs required")
    def test_non_can_interface_is_rejected_before_sdk_connect(self):
        self.options["channel"] = "lo"
        self.plan["device"]["arm"]["resource_id"] = "agx_lo"
        with self.assertRaisesRegex(RuntimeError, "SocketCAN lo is not a CAN interface"):
            DeviceClient(CLI, self.filename, allow_motion=False)
        self.assertEqual(self.events(), [])

    def test_full_worker_point_roundtrip(self):
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status":"succeeded"', result.stdout)

    def test_dropped_target_is_resent_unchanged(self):
        # The controller ignored the first frames and reports idle away from the target: the
        # worker re-sends the same target, and the point completes on fresh evidence.
        self.channel("can_drop")
        self.plan["steps"] = [
            {"capability": "joint_point", "target_rad": [0.5] * 7, "speed_scale": 0.05}
        ]
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status":"succeeded"', result.stdout)
        self.assertIn("re-sent the same target (1/5)", result.stderr)
        moves = [
            event["target"]
            for event in map(
                json.loads,
                (self.sdk / "pyAgxArm/recovery-events.jsonl")
                .read_text(encoding="utf-8")
                .splitlines(),
            )
            if event["op"] == "move_j"
        ]
        self.assertEqual(moves, [[0.5] * 7, [0.5] * 7])

    def test_point_resent_at_interval_completes_within_tolerance(self):
        # Controller-endpoint placement: the same target re-sent every 50 ms while off target,
        # complete on two fresh samples within tolerance, before the arm is seen still.
        self.channel("can_drop")
        with DeviceClient(CLI, self.filename) as client:
            started = time.monotonic()
            result = client.move_arm(
                [0.5] * 7,
                1.0,
                tolerance_rad=math.radians(0.5),
                wait_for_arrival=False,
                resend_interval_s=0.05,
            )
            self.assertLess(time.monotonic() - started, 0.5, "not the 1 s stationary re-send")
            self.assertIn("within_tolerance", result["detail"])
            for interval in (0.01, 0.3, -0.05):
                with self.assertRaises(DeviceCommandError):
                    client.move_arm([0.0] * 7, 1.0, resend_interval_s=interval)
            self.assertFalse(client.state()["fault"])
        moves = [event["target"] for event in self.recorded() if event["op"] == "move_j"]
        self.assertEqual(moves, [[0.5] * 7, [0.5] * 7])

    def test_state_after_idle_does_not_wait_for_the_motion_window(self):
        # Joint feedback is recorded in the background: a read after an idle gap is answered at
        # once.
        with DeviceClient(CLI, self.filename) as client:
            durations = []
            for _ in range(3):
                time.sleep(0.3)
                started = time.monotonic()
                self.assertFalse(client.state()["moving"])
                durations.append(time.monotonic() - started)
        self.assertLess(min(durations), 0.1, durations)

    def recorded(self):
        trace = self.sdk / "pyAgxArm/recovery-events.jsonl"
        return (
            [json.loads(line) for line in trace.read_text(encoding="utf-8").splitlines()]
            if trace.exists()
            else []
        )

    def test_joint_path_streams_one_smooth_motion(self):
        # One continuous move_js stream through every waypoint, without stopping between them;
        # js mode is set once and the SDK's automatic mode switching is restored afterwards.
        self.channel("can_path")
        waypoints = [[0.1] * 7, [0.1, 0.2, 0.1, 0.1, 0.1, 0.1, 0.1], [0.3] * 7]
        with DeviceClient(CLI, self.filename) as client:
            result = client.move_arm_path(waypoints, 0.4, 0.35, 1.0)
            self.assertEqual(result["status"], "succeeded", result)
            self.assertEqual(client.state()["joints_rad"], [0.3] * 7)
        events = self.recorded()
        streamed = [e["target"] for e in events if e["op"] == "move_js"]
        # js mode is set once, explicitly; the SDK's own per-command mode switching stays off.
        self.assertEqual(
            [e for e in events if e["op"] == "set_motion_mode"],
            [{"op": "set_motion_mode", "mode": "js"}],
        )
        self.assertEqual([e["enabled"] for e in events if e["op"] == "auto_motion_mode"], [False])
        self.assertNotIn("move_j", [e["op"] for e in events])
        self.assertGreater(len(streamed), 20)
        self.assertEqual(streamed[-1], [0.3] * 7)
        steps = [
            max(abs(a - b) for a, b in zip(p, q)) for p, q in zip([[0.0] * 7] + streamed, streamed)
        ]
        self.assertLessEqual(max(steps), math.radians(0.5))
        self.assertTrue(
            any(abs(q[1] - 0.2) < 0.01 for q in streamed), "the path passes the middle waypoint"
        )

    def test_stream_steady_state_error_within_path_tolerance_completes_in_js_mode(self):
        # js mode stops a few tenths of a degree short; the path's own tolerance (0.8 deg) accepts
        # that, without switching modes or re-sending.
        self.channel("can_offset")
        with DeviceClient(CLI, self.filename) as client:
            result = client.move_arm_path(
                [[0.2] * 7], 0.4, 0.35, 1.0, tolerance_rad=math.radians(0.8)
            )
            self.assertEqual(result["status"], "succeeded")
            self.assertAlmostEqual(client.state()["joints_rad"][4], 0.203)
            with self.assertRaises(DeviceCommandError):  # Beyond the 0.8 deg ceiling.
                client.move_arm_path([[0.1] * 7], 0.4, 0.35, 1.0, tolerance_rad=math.radians(1))
        operations = [e["op"] for e in self.recorded()]
        self.assertNotIn("move_j", operations)
        self.assertEqual(operations.count("set_motion_mode"), 1)

    def test_stationary_arm_short_of_target_is_re_sent_despite_controller_flag(self):
        # The controller keeps motion_status=1 ("target not reached") while the arm stands still:
        # motion is measured from joint feedback, so the worker re-sends and the point completes.
        self.channel("can_notreached")
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("re-sent the same target (1/5)", result.stderr)

    def test_cancelled_path_stops_streaming_then_holds(self):
        self.channel("can_path")
        started = []
        with DeviceClient(CLI, self.filename) as client:

            def cancelled():
                started.append(time.monotonic())
                return time.monotonic() - started[0] > 1.0

            with self.assertRaises(DeviceCommandError) as raised:
                client.move_arm_path([[0.9] * 7], 0.4, 0.05, 0.05, is_cancelled=cancelled)
            self.assertEqual(
                raised.exception.result["status"], "cancelled", raised.exception.result
            )
            position = client.state()["joints_rad"]
        self.assertTrue(0 < position[0] < 0.9)
        operations = [e["op"] for e in self.recorded()]
        hold = operations.index("move_j")
        self.assertNotIn("move_js", operations[hold:], "no stream step after the hold")
        self.assertGreater(operations[:hold].count("move_js"), 10)

    def test_command_to_the_resting_pose_sends_nothing(self):
        # Re-sending the current pose would only re-plan or switch the controller mode.
        self.channel("can_path")
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(client.move_arm([0.0005] * 7, 0.05)["status"], "succeeded")
            self.assertEqual(
                client.move_arm_path([[0.001] * 7, [0.0] * 7], 0.4, 0.35, 1.0)["status"],
                "succeeded",
            )
        operations = {e["op"] for e in self.recorded()}
        self.assertFalse(
            {"move_j", "move_js", "set_motion_mode", "set_speed_percent"} & operations, operations
        )

    def test_controller_limits_are_reported(self):
        with DeviceClient(CLI, self.filename) as client:
            limits = client.state()["controller_limits"]
        self.assertEqual([row["joint"] for row in limits], list(range(1, 8)))
        self.assertEqual([row["max_velocity_rad_s"] for row in limits], [3.14] * 4 + [3.92] * 3)
        self.assertEqual({row["max_acceleration_rad_s2"] for row in limits}, {5.0})

    def test_timed_trajectory_is_streamed_as_given(self):
        # A 1 s shake-like profile on joint 7 at 200 Hz, from the current position and back.
        self.channel("can_path")
        times = [i / 100 for i in range(101)]
        positions = [[0.0] * 6 + [0.02 * math.sin(2 * math.pi * t)] for t in times]
        with DeviceClient(CLI, self.filename) as client:
            result = client.move_arm_trajectory(
                times,
                positions,
                command_rate_hz=200,
                max_step_rad=math.radians(1),
                tolerance_rad=math.radians(0.5),
            )
            self.assertEqual(result["status"], "succeeded", result)
            # The shake recipe's own bound, exactly 3 degrees, survives serialization to the worker.
            result = client.move_arm_trajectory(
                times,
                positions,
                command_rate_hz=200,
                max_step_rad=math.radians(3),
                tolerance_rad=math.radians(0.5),
            )
            self.assertEqual(result["status"], "succeeded", result)
            for bad in (
                dict(positions=[[0.1] * 7] + positions[1:]),  # Does not start at the arm.
                dict(
                    positions=positions[:50] + [[0.0] * 6 + [0.2]] + positions[51:]
                ),  # Step too large.
                dict(max_step_rad=math.radians(4)),
            ):
                arguments = dict(
                    times_s=times,
                    positions_rad=positions,
                    command_rate_hz=200,
                    max_step_rad=math.radians(1),
                )
                arguments.update(
                    {("positions_rad" if k == "positions" else k): v for k, v in bad.items()}
                )
                with self.assertRaises(DeviceCommandError):
                    client.move_arm_trajectory(**arguments)
            self.assertFalse(client.state()["fault"])
        events = self.recorded()
        streamed = [e["target"] for e in events if e["op"] == "move_js"]
        self.assertGreater(len(streamed), 150)  # About 200 Hz for 1 s.
        self.assertAlmostEqual(max(q[6] for q in streamed), 0.02, places=3)
        self.assertEqual([e["mode"] for e in events if e["op"] == "set_motion_mode"], ["js"])

    def test_trajectory_without_arrival_wait_hands_over_while_the_arm_settles(self):
        # The command completes once its last target is sent, and the next one starts from the
        # measured position while the arm still converges.
        self.channel("can_lag")
        times = [i / 100 for i in range(31)]
        out = [[0.1 * t / 0.3] * 7 for t in times]
        with DeviceClient(CLI, self.filename) as client:
            result = client.move_arm_trajectory(
                times,
                out,
                command_rate_hz=100,
                tolerance_rad=math.radians(0.5),
                wait_for_arrival=False,
            )
            self.assertEqual(result["status"], "succeeded")
            self.assertIn("delivered", result["detail"])
            state = client.state()
            self.assertTrue(state["moving"])
            self.assertGreater(
                abs(0.1 - state["joints_rad"][0]), math.radians(0.5), "still short of the target"
            )
            here = state["joints_rad"]
            back = [[a + (0.0 - a) * t / 0.3 for a in here] for t in times]
            result = client.move_arm_trajectory(
                times, back, command_rate_hz=100, tolerance_rad=math.radians(0.5)
            )
            self.assertIn("target_reached", result["detail"])  # A waiting command still waits.
            self.assertLess(max(abs(q) for q in client.state()["joints_rad"]), math.radians(0.5))

    def test_follow_replaces_the_target_while_the_arm_moves(self):
        self.channel("can_lag")
        with DeviceClient(CLI, self.filename) as client:
            for step in (0.05, 0.1, 0.15):
                result = client.follow_arm([step] * 7, 0.5)
                self.assertEqual((result["status"], result["detail"][:4]), ("succeeded", "sent"))
            self.assertTrue(client.state()["moving"])
            trace = (
                (self.sdk / "pyAgxArm/recovery-events.jsonl")
                .read_text(encoding="utf-8")
                .splitlines()
            )
            moves = [event["target"] for event in map(json.loads, trace) if event["op"] == "move_j"]
            self.assertEqual(moves[-3:], [[0.05] * 7, [0.1] * 7, [0.15] * 7])
            self.assertIn(
                "target_reached",
                client.move_arm([0.15] * 7, 0.5, tolerance_rad=math.radians(0.5))["detail"],
            )

    def test_revo2_uses_the_framework_scale_of_zero_closed_one_open(self):
        names = [
            "thumb_tip",
            "thumb_base",
            "index_finger",
            "middle_finger",
            "ring_finger",
            "pinky_finger",
        ]
        self.plan["device"]["effector"] = dict(
            device_id="hand_main", channels=names, feedback="position"
        )
        self.channel("can_fixture")
        with DeviceClient(CLI, self.filename) as client:
            state = client.state()
            self.assertEqual(state["effector_positions"], [1] * 6)  # The fixture hand starts open.
            self.assertEqual(
                client.move_effector([0.7] * 6, duration_s=0.05)["status"], "succeeded"
            )
            hand = client.state()["effector_positions"]
            self.assertTrue(all(abs(value - 0.7) < 0.01 for value in hand), hand)

    def test_path_outside_limits_writes_nothing(self):
        self.channel("can_path")
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError):
                client.move_arm_path([[0.5] * 7, [1.5] + [0.0] * 6], 0.4, 0.35, 1.0)
            with self.assertRaises(DeviceCommandError):
                client.move_arm_path(
                    [[0.5] * 7], 0.4, 0.5, 1.0
                )  # Faster than the streaming ceiling.
            self.assertFalse(client.state()["fault"])
        self.assertFalse({"move_js", "set_motion_mode"} & {e["op"] for e in self.recorded()})

    def test_relative_resources_and_path_interpreter_from_another_directory(self):
        self.options.update(
            sdk_root="sdk", worker=os.path.relpath(WORKER, self.directory), python="python3"
        )
        elsewhere = self.directory / "unrelated working directory"
        elsewhere.mkdir()
        result = self.call("run", cwd=elsewhere)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('"status":"succeeded"', result.stdout)

    def test_supervised_arm_check_against_fake_sdk(self):
        for extra in ([], ["--timeout-only"]):
            with self.subTest(extra=extra):
                result = self.call("arm-check", *extra)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("timeout=timed_out" if extra else "cancel=cancelled", result.stdout)

    def test_motion_needs_no_flag_and_read_only_service_refuses_it(self):
        self.assertEqual(
            self.call("run", "--allow-motion").returncode, 2, "the removed opt-in is not accepted"
        )
        with DeviceClient(CLI, self.filename, allow_motion=False) as client:
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.05] * 7, 0.05)
        self.assertNotIn("move_j", self.events())

    def test_stale_and_partial_feedback_rejected(self):
        for channel in ("can_stale", "can_partial"):
            with self.subTest(channel=channel):
                self.channel(channel)
                self.assertEqual(self.call("inspect").returncode, 2)

    def test_controller_and_motor_errors_prevent_motion(self):
        for channel, marker in (
            ("can_armfault", '"fault":true'),
            ("can_motorfault", '"fault":true'),
            ("can_badmode", "position mode"),
        ):
            with self.subTest(channel=channel):
                self.channel(channel)
                result = self.call("run")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(marker, result.stdout + result.stderr)

    def test_legacy_can_lease_prevents_connection(self):
        with open(
            f"/tmp/nero_{self.options['channel']}_control.lock", "a", encoding="utf-8"
        ) as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(self.call("inspect").returncode, 2)

    def test_sdk_write_failure_is_not_reported_stopped(self):
        self.channel("can_writefail")
        result = self.call("run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('"status":"stop_unknown"', result.stdout)

    def test_driver_errors_and_joint_limits(self):
        self.plan["steps"][0]["speed_scale"] = 0.005
        self.assertEqual(self.call("validate").returncode, 2)
        self.plan["steps"][0]["speed_scale"] = 0.05
        self.plan["steps"][0]["target_rad"][0] = 2
        self.assertEqual(self.call("validate").returncode, 2)
        self.plan["steps"] = []
        self.plan["device"]["driver"] = "missing"
        self.assertEqual(self.call("validate").returncode, 2)

    def test_rpc_corruption_timeout_and_eof_fail_closed(self):
        self.options["worker"] = str(ROOT / "tests/fault_worker.py")
        for channel in (
            "can_wrongid",
            "can_wrongv",
            "can_eof",
            "can_oversized",
            "can_half",
            "can_timeout",
        ):
            with self.subTest(channel=channel):
                self.channel(channel)
                start = time.monotonic()
                self.assertEqual(self.call("inspect").returncode, 2)
                self.assertLess(time.monotonic() - start, 5)

    def test_refusal_before_write_keeps_the_connection(self):
        # A disabled arm refuses motion in the worker before any CAN write. The owner keeps the
        # connection: the same serve process still answers, and nothing is latched.
        self.channel("can_disabled")
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError) as raised:
                client.move_arm([0.05] * 7, 0.05)
            self.assertNotIn("stop_unknown", str(raised.exception))
            self.assertFalse(client.state()["enabled"])
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.05] * 7, 0.05)
            self.assertFalse(client.state()["fault"])

    def test_terminal_and_service_signals_reach_the_hold(self):
        # Ctrl+C in a terminal and systemd stop signal the whole process group. The vendor worker
        # must survive long enough to carry the hold, so the result is a verified cancel.
        for name in ("SIGINT", "SIGTERM"):
            with self.subTest(signal=name):
                self.channel("can_stall")
                trace = self.sdk / "pyAgxArm/recovery-events.jsonl"
                trace.unlink(missing_ok=True)
                self.plan["steps"] = [
                    {"capability": "joint_point", "target_rad": [0.5] * 7, "speed_scale": 0.05}
                ]
                self.plan["device"]["arm"]["timeout_s"] = 20
                process = subprocess.Popen(
                    [CLI, "run", str(self.filename)],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    start_new_session=True,
                )
                deadline = time.monotonic() + 8
                while self.events().count("move_j") < 1 and time.monotonic() < deadline:
                    time.sleep(0.02)
                time.sleep(0.2)
                os.killpg(process.pid, getattr(signal, name))
                stdout, stderr = process.communicate(timeout=15)
                self.assertIn('"status":"cancelled"', stdout, stdout + stderr)
                operations = self.events()
                moves = [
                    json.loads(line)["target"]
                    for line in trace.read_text(encoding="utf-8").splitlines()
                    if json.loads(line)["op"] == "move_j"
                ]
                # The target (re-sent while the controller stays idle away from it), then the hold.
                self.assertEqual(moves[0], [0.5] * 7)
                self.assertEqual(moves[-1], [0.0] * 7, "the hold retargets the sampled position")
                self.assertTrue(all(move == [0.5] * 7 for move in moves[:-1]))
                self.assertLess(
                    len(operations) - 1 - operations[::-1].index("move_j"),
                    operations.index("disconnect"),
                    "hold was sent after the SDK closed",
                )

    def test_inflight_fault_latches_and_reconnect_never_replays(self):
        self.channel("recover")
        for scenario in (
            "worker_exit",
            "rpc_timeout",
            "truncated",
            "fault",
            "motor",
            "disabled",
            "mode",
            "stale",
            "part",
        ):
            with self.subTest(scenario=scenario):
                (self.sdk / "pyAgxArm/recovery-events.jsonl").unlink(missing_ok=True)
                result = subprocess.run(
                    [RECOVERY, str(self.filename), scenario],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(
                    "result=stop_unknown latched=1 reconnect=idle actual_state_verified=1",
                    result.stdout,
                )
                operations = self.events()
                self.assertEqual(operations.count("connect"), 2)
                self.assertEqual(
                    operations.count("move_j"), 1, "no hold target or reconnect replay"
                )
                self.assertNotIn("enable", operations)

    def test_recovery_helper_rejects_missing_fixture_marker_before_connect(self):
        self.channel("recover")
        (self.sdk / "TEST_FIXTURE_ONLY").unlink()
        result = subprocess.run(
            [RECOVERY, str(self.filename), "worker_exit"], capture_output=True, text=True, timeout=5
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("refusing non-fixture SDK", result.stderr)
        self.assertFalse((self.sdk / "pyAgxArm/recovery-events.jsonl").exists())


class SocketCanStateTests(unittest.TestCase):
    def test_down_interface_is_rejected_without_changing_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            interface = Path(temporary) / "can0"
            interface.mkdir()
            (interface / "type").write_text("280\n")
            (interface / "flags").write_text("0x80\n")
            with self.assertRaisesRegex(RuntimeError, "SocketCAN can0 is DOWN"):
                worker.check_socketcan_state("can0", Path(temporary))
            self.assertEqual((interface / "flags").read_text(), "0x80\n")

    def test_up_interface_does_not_depend_on_operstate(self):
        with tempfile.TemporaryDirectory() as temporary:
            interface = Path(temporary) / "can0"
            interface.mkdir()
            (interface / "type").write_text("280\n")
            (interface / "flags").write_text("0x81\n")
            (interface / "operstate").write_text("unknown\n")
            worker.check_socketcan_state("can0", Path(temporary))

    def test_missing_sysfs_leaves_connection_validation_to_sdk(self):
        with tempfile.TemporaryDirectory() as temporary:
            worker.check_socketcan_state("can_fixture", Path(temporary))


class PathProfileTests(unittest.TestCase):
    def test_profile_ends_at_rest_on_each_endpoint_and_never_jumps(self):
        points = [[0.0] * 7, [0.2] + [0.0] * 6, [0.2, 0.3] + [0.0] * 5]
        profile = worker.path_profile(points, math.radians(20), math.radians(20))
        self.assertAlmostEqual(profile[1][-1], 0.5)
        self.assertAlmostEqual(
            profile[2],
            max(0.3, 1.875 * 0.5 / math.radians(20), math.sqrt(5.774 * 0.5 / math.radians(20))),
        )
        self.assertEqual(worker.path_target(profile, 0), points[0])
        self.assertEqual(worker.path_target(profile, profile[2]), points[-1])
        times = [profile[2] * i / 2000 for i in range(2001)]
        targets = [worker.path_target(profile, t) for t in times]
        travelled = [abs(q[0]) + abs(q[1]) for q in targets]
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(travelled, travelled[1:])))
        speed = [
            max(abs(a - b) for a, b in zip(p, q)) / (times[1] - times[0])
            for p, q in zip(targets, targets[1:])
        ]
        self.assertLessEqual(max(speed), math.radians(20) * 1.001)
        self.assertLess(speed[0], 1e-3)
        self.assertLess(speed[-1], 1e-3)

    def test_stale_feedback_stops_the_stream_and_retargets_feedback(self):
        class Robot:
            def __init__(self):
                self.sent, self.automatic = [], False

            def get_joint_angles(self):
                return Node(timestamp=1.0, msg=[0.0] * 7)  # Never refreshes.

            def move_js(self, target):
                self.sent.append(list(target))

            def set_auto_set_motion_mode_enabled(self, enabled):
                self.automatic = enabled

        connection = Node(robot=Robot(), sdk_lock=threading.Lock())
        profile = worker.path_profile([[0.0] * 7, [0.5] * 7], 0.05, 0.05)
        stream = worker.Stream(
            connection,
            lambda elapsed: worker.path_target(profile, elapsed),
            profile[2],
            rate=50,
            max_step=math.radians(0.5),
            max_gap=0.1,
            advance=0.02,
            final=[0.5] * 7,
            tolerance=0.01,
        )
        stream.run()
        self.assertIn("stale", stream.error)
        self.assertFalse(stream.done)
        self.assertEqual(
            connection.robot.sent[-1], [0.0] * 7, "the last command retargets feedback"
        )


class ProtocolTests(unittest.TestCase):
    class Connection:
        def __init__(self, fail=None, write=False):
            self.calls, self.fail, self.write_first, self.wrote = 0, fail, write, False

        def read(self):
            self.calls += 1
            if self.write_first:
                self.wrote = True
            if self.fail:
                raise RuntimeError(self.fail)
            return {}

        connect = read
        diagnose = read
        enable = read
        hold = read
        poll = read

    def request(self, **updates):
        result = {
            "v": 1,
            "session": "test",
            "id": 1,
            "deadline_ns": time.monotonic_ns() + 10**9,
            "op": "read",
        }
        result.update(updates)
        return json.dumps(result).encode() + b"\n"

    def test_expired_and_unknown_ops_do_not_dispatch(self):
        for updates in ({"deadline_ns": 0}, {"op": "__getattr__"}, {"extra": True}):
            connection, writer = self.Connection(), io.BytesIO()
            worker.serve(connection, io.BytesIO(self.request(**updates)), writer)
            self.assertEqual(connection.calls, 0)
            response = json.loads(writer.getvalue())
            self.assertFalse(response["ok"])
            self.assertTrue(response["rejected"])

    def test_failure_after_a_write_is_not_a_rejection(self):
        for write, rejected in ((False, True), (True, False)):
            writer = io.BytesIO()
            worker.serve(
                self.Connection(fail="refused", write=write), io.BytesIO(self.request()), writer
            )
            self.assertEqual(json.loads(writer.getvalue())["rejected"], rejected)

    def test_replay_and_session_change_are_rejected(self):
        for second in (self.request(), self.request(id=2, session="other")):
            with self.assertRaises(RuntimeError):
                worker.serve(self.Connection(), io.BytesIO(self.request() + second), io.BytesIO())

    def test_partial_oversize_and_duplicate_frames(self):
        for frame in (b"{", b"x" * (worker.MAX_FRAME + 1), b'{"v":1,"v":1}\n'):
            with self.assertRaises((RuntimeError, ValueError)):
                worker.serve(self.Connection(), io.BytesIO(frame), io.BytesIO())


if __name__ == "__main__":
    unittest.main()
