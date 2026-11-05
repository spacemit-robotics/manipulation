# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Assembly, local binding and shared CAN-free vendor session contract tests."""

import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))
# Imported after the source binding path is added.
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402

CLI = os.environ["MANIPULATION_DEVICE"]


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="device-contract-")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.plan = dict(
            schema_version=3,
            device=dict(
                driver="mock_device",
                arm=dict(
                    device_id="arm",
                    resource_id="mock_assembly",
                    joints=[
                        dict(name="j1", minimum_rad=-1, maximum_rad=1),
                        dict(name="j2", minimum_rad=-1, maximum_rad=1),
                    ],
                ),
                effector=dict(device_id="hand", channels=["a", "b", "c", "d", "e", "f"]),
            ),
            steps=[dict(capability="joint_point", target_rad=[0.1, -0.1], speed_scale=0.05)],
        )
        self.filename = self.path / "device.json"
        self.save()

    def save(self):
        self.filename.write_text(json.dumps(self.plan), encoding="utf-8")

    def cli(self, *args):
        return subprocess.run([CLI, *args], text=True, capture_output=True, timeout=12)

    def client(self, motion=True):
        return DeviceClient(CLI, self.filename, allow_motion=motion)

    def test_mock_plan_runs_without_an_opt_in(self):
        self.assertEqual(self.cli("validate", str(self.filename)).returncode, 0)
        self.assertNotEqual(self.cli("run", str(self.filename), "--allow-motion").returncode, 0)
        self.assertNotEqual(self.cli("run", str(self.filename), "--read-only").returncode, 0)
        result = self.cli("run", str(self.filename))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"physical_grip_verified":false', result.stdout)

    def test_binding_readonly_then_motion_and_exclusive_lease(self):
        with self.client(False) as client:
            self.assertEqual(client.state()["joints_rad"], [0, 0])
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1, 0.1], 0.1)
            result = self.cli("inspect", str(self.filename))
            self.assertNotEqual(result.returncode, 0)
        with self.client() as client:
            client.move_arm([0.1, -0.1], 0.1)
            client.move_effector([0.2] * 6)
            self.assertEqual(client.state()["effector_positions"], [0.2] * 6)
            self.assertEqual(client.state()["joints_rad"], [0.1, -0.1])

    def test_lease_is_host_wide_across_working_directories(self):
        elsewhere = self.path / "another directory"
        elsewhere.mkdir()
        with self.client():
            result = subprocess.run(
                [CLI, "inspect", str(self.filename)],
                cwd=elsewhere,
                text=True,
                capture_output=True,
                timeout=12,
            )
            self.assertNotEqual(
                result.returncode, 0, "a second working directory acquired the same device"
            )
            self.assertIn("busy", result.stderr)
        result = subprocess.run(
            [CLI, "inspect", self.filename.name],
            cwd=self.path,
            text=True,
            capture_output=True,
            timeout=12,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_open_loop_effector_is_unverified(self):
        self.plan["device"]["effector"]["feedback"] = "none"
        self.plan["steps"] = [
            dict(capability="effector_position", positions=[0.3] * 6, duration_s=0.1),
            dict(capability="joint_point", target_rad=[0.1, -0.1], speed_scale=0.05),
        ]
        self.save()
        self.assertEqual(
            json.loads(self.cli("describe", str(self.filename)).stdout)["effector_feedback"], "none"
        )
        result = self.cli("run", str(self.filename))
        self.assertEqual(result.returncode, 3, result.stdout + result.stderr)
        self.assertIn('"status":"unverified"', result.stdout)
        with self.client() as client:
            state = client.state()
            self.assertEqual(
                (state["effector_feedback"], state["effector_positions"]), ("none", [])
            )
            with self.assertRaises(DeviceCommandError) as raised:
                client.move_effector([0.2] * 6, duration_s=0.1)
            self.assertEqual(raised.exception.result["status"], "unverified")
            result = client.move_effector([0.2] * 6, duration_s=0.1, allow_unverified=True)
            self.assertEqual(result["status"], "unverified")
            client.move_arm([0.0, 0.0], 0.05)  # Not latched by an unverified move.

    def test_describe_is_offline_metadata(self):
        described = json.loads(self.cli("describe", str(self.filename)).stdout)
        self.assertEqual(described.pop("joint_limits_rad"), [[-1, 1], [-1, 1]])
        self.assertEqual(
            described.pop("arm_timeout_s"), self.plan["device"]["arm"].get("timeout_s", 5)
        )
        self.assertEqual(
            described,
            dict(
                driver="mock_device",
                model="",
                joints=["j1", "j2"],
                channels=["a", "b", "c", "d", "e", "f"],
                effector_feedback="position",
            ),
        )

    def test_startup_error_retains_the_cause_instead_of_eof(self):
        self.plan["device"]["driver"] = "missing"
        self.save()
        result = self.cli("serve", str(self.filename))
        self.assertEqual(result.returncode, 2)
        error = json.loads(result.stdout)
        self.assertEqual((error["v"], error["id"], error["ok"]), (1, 0, False))
        self.assertIn("unknown device driver", error["error"])
        for _ in range(
            3
        ):  # Both the structured error and immediate EOF may arrive before the waiter.
            with self.assertRaisesRegex(RuntimeError, "unknown device driver"):
                self.client(False)

    def test_cancel_before_submission_and_bad_targets(self):
        with self.client(True) as client:
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1, 0.1], 0.1, is_cancelled=lambda: True)
            self.assertEqual(client.state()["joints_rad"], [0, 0])
            for positions in ([1.1] * 6, [0] * 5):
                with self.assertRaises(DeviceCommandError):
                    client.move_effector(positions)

    def test_schema_rejects_unknown_and_tool_without_tool(self):
        self.plan["device"]["typo"] = 1
        self.save()
        self.assertNotEqual(self.cli("validate", str(self.filename)).returncode, 0)
        del self.plan["device"]["typo"]
        del self.plan["device"]["effector"]
        self.plan["steps"] = [dict(capability="effector_position", positions=[], duration_s=0.5)]
        self.save()
        self.assertNotEqual(self.cli("validate", str(self.filename)).returncode, 0)

    def test_step_longer_than_the_effector_timeout_is_invalid(self):
        self.plan["steps"] = [
            dict(capability="effector_position", positions=[0.5] * 6, duration_s=6)
        ]
        self.save()
        result = self.cli("validate", str(self.filename))
        self.assertNotEqual(result.returncode, 0)
        self.plan["steps"][0]["duration_s"] = 0.5
        self.save()
        self.assertEqual(self.cli("validate", str(self.filename)).returncode, 0)

    def test_signal_exits_service_without_hanging(self):
        client = self.client()
        client.process.send_signal(signal.SIGINT)
        self.assertEqual(client.process.wait(timeout=3), 0)
        client.close()

    def test_lost_response_pipe_closes_owner_without_more_commands(self):
        process = subprocess.Popen(
            [CLI, "serve", str(self.filename)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        try:
            self.assertEqual(json.loads(process.stdout.readline())["event"], "ready")
            process.stdout.close()
            process.stdin.write(
                json.dumps(
                    dict(v=1, id=1, op="state", deadline_ns=time.monotonic_ns() + 2_000_000_000)
                )
                + "\n"
            )
            process.stdin.flush()
            self.assertEqual(process.wait(timeout=3), 0)
        finally:
            process.stdin.close()
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def nero(self, mode="recover"):
        sdk = self.path / "sdk"
        shutil.copytree(ROOT / "tests/fake_sdk", sdk)
        (sdk / "UPSTREAM_COMMIT").write_text(
            "e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b\n", encoding="utf-8"
        )
        self.plan["device"]["driver"] = "agx_nero_revo2"
        arm = self.plan["device"]["arm"]
        arm["joints"] = [dict(name=f"joint{i}", minimum_rad=-1, maximum_rad=1) for i in range(1, 8)]
        channel = f"t{os.getpid() % 0x10000:04x}_{mode}"
        arm["resource_id"] = "agx_" + channel
        self.plan["device"]["options"] = dict(
            worker=str(ROOT / "drivers/common/agx_session/agx_worker.py"),
            sdk_root=str(sdk),
            python=sys.executable,
            channel=channel,
        )
        self.plan["device"]["effector"]["channels"] = [
            "thumb_tip",
            "thumb_base",
            "index_finger",
            "middle_finger",
            "ring_finger",
            "pinky_finger",
        ]
        self.plan["steps"] = []
        (sdk / "pyAgxArm/recovery.json").write_text('{"mode":"ok"}', encoding="utf-8")
        self.save()
        return sdk

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_shared_arm_hand_vendor_worker_and_validation(self):
        sdk = self.nero()
        with self.client(True) as client:
            client.move_arm([0.01] * 7, 0.05)
            client.move_effector([0.1] * 6)
            for value in client.state()["effector_positions"]:  # Command 10/100 reads back 26/255.
                self.assertAlmostEqual(value, 0.1, delta=0.005)
        events = [
            json.loads(line)["op"]
            for line in (sdk / "pyAgxArm/recovery-events.jsonl").read_text().splitlines()
        ]
        self.assertEqual(events.count("connect"), 1)
        self.assertEqual(events.count("init_effector"), 1)
        self.assertEqual(events.count("disconnect"), 1)
        self.assertNotIn("enable", events)
        self.assertEqual(events.count("hand_pos"), 1)
        self.assertEqual(events.count("hand_time"), 1)

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_gesture_sends_hand_targets_while_the_arm_streams(self):
        # Result gestures: hand targets on schedule while the arm moves (open-loop hand).
        sdk = self.nero("hnofb")
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        times = [i / 100 for i in range(31)]
        with self.client() as client:
            result = client.move_arm_trajectory(
                times,
                [[0.1 * t / 0.3] * 7 for t in times],
                command_rate_hz=100,
                tolerance_rad=math.radians(0.5),
                wait_for_arrival=False,
                effector_schedule=[(0, [1, 1, 0, 0, 1, 1], 0.01), (0.1, [0] * 6, 0.01)],
                allow_unverified=True,
            )
            self.assertEqual(result["status"], "unverified", result)
            self.assertFalse(client.state()["fault"])
        events = [
            json.loads(line)["op"]
            for line in (sdk / "pyAgxArm/recovery-events.jsonl").read_text().splitlines()
        ]
        streamed = [i for i, op in enumerate(events) if op == "move_js"]
        hands = [i for i, op in enumerate(events) if op == "hand_pos"]
        self.assertEqual(len(hands), 2)
        self.assertLess(hands[0], streamed[-1], "the hand did not move while the arm streamed")

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_grip_while_the_arm_settles_toward_its_target(self):
        # FAST hand-over: the trajectory completes once delivered and the hand closes while the arm,
        # which trails its targets, still converges by more than 1 degree; converging is not a
        # disturbance.
        self.nero("lag")
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        times = [i / 100 for i in range(21)]
        with self.client() as client:
            result = client.move_arm_trajectory(
                times,
                [[0.2 * t / 0.2] * 7 for t in times],
                command_rate_hz=100,
                tolerance_rad=math.radians(0.5),
                wait_for_arrival=False,
            )
            self.assertIn("delivered", result["detail"])
            behind = 0.2 - client.state()["joints_rad"][0]
            self.assertGreater(behind, math.radians(1.5), "the fixture arm must still be settling")
            result = client.move_effector([0.5] * 6, duration_s=0.3, allow_unverified=True)
            self.assertEqual(result["status"], "unverified", result)
            self.assertLess(abs(0.2 - client.state()["joints_rad"][0]), math.radians(0.5))

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_relative_shared_vendor_resources(self):
        self.nero()
        options = self.plan["device"]["options"]
        options["sdk_root"] = "sdk"
        options["worker"] = os.path.relpath(options["worker"], self.path)
        options["python"] = "python3"
        self.save()
        elsewhere = self.path / "another directory"
        elsewhere.mkdir()
        result = subprocess.run(
            [CLI, "inspect", str(self.filename)],
            cwd=elsewhere,
            text=True,
            capture_output=True,
            timeout=12,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_revo_without_position_frames_runs_open_loop(self):
        # The observed Revo2 publishes only finger current. position feedback fails closed; none
        # runs open loop.
        sdk = self.nero("hnofb")
        self.assertNotEqual(self.cli("inspect", str(self.filename)).returncode, 0)
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        with self.client() as client:
            state = client.state()
            self.assertEqual(
                (state["effector_feedback"], state["effector_positions"]), ("none", [])
            )
            result = client.move_effector([0.1] * 6, duration_s=0.2, allow_unverified=True)
            self.assertEqual(result["status"], "unverified")
            self.assertFalse(result["physical_grip_verified"])
        events = [
            json.loads(line)["op"]
            for line in (sdk / "pyAgxArm/recovery-events.jsonl").read_text().splitlines()
        ]
        self.assertEqual((events.count("hand_pos"), events.count("hand_time")), (1, 1))
        sdk.rename(self.path / "sdk_nofb")
        self.nero("hdead")
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        self.assertNotEqual(
            self.cli("inspect", str(self.filename)).returncode, 0, "absent hand accepted"
        )

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_revo_feedback_preconditions_and_unverified_stop_latch(self):
        for mode in ("hleft", "hstale", "hmissing", "hstall", "hnoend"):
            with self.subTest(mode=mode):
                # Each fixture gets its own SDK source; never reuse a failed worker.
                sdk = self.path / "sdk"
                if sdk.exists():
                    sdk.rename(self.path / ("sdk_" + mode))
                self.nero(mode)
                self.plan["device"]["effector"]["timeout_s"] = 0.08
                self.save()
                if mode in ("hleft", "hstale", "hmissing"):
                    self.assertNotEqual(self.cli("inspect", str(self.filename)).returncode, 0)
                else:
                    with self.client(True) as client:
                        with self.assertRaises(DeviceCommandError) as raised:
                            client.move_effector([0.5] * 6, duration_s=0.05)  # Within timeout_s.
                        if mode == "hnoend":
                            self.assertIn("stop_unknown", str(raised.exception))
                        with self.assertRaises(DeviceCommandError):
                            client.move_arm([0] * 7, 0.05)


if __name__ == "__main__":
    unittest.main()
