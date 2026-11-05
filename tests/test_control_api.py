# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Public control operations and permissions over the real local service, without hardware."""

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402

CLI = os.environ["MANIPULATION_DEVICE"]
SPEC = importlib.util.spec_from_file_location(
    "spacemit_manipulation.operator_cli", ROOT / "capabilities/diagnostics/cli/operator_cli.py"
)
operator_cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(operator_cli)


class ControlApiTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="control-api-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.plan = {
            "schema_version": 3,
            "device": {
                "driver": "mock_device",
                "arm": {
                    "device_id": "arm",
                    "resource_id": "control_api_binding",
                    "joints": [
                        {"name": "j1", "minimum_rad": -1, "maximum_rad": 1},
                        {"name": "j2", "minimum_rad": -1, "maximum_rad": 1},
                    ],
                },
                "effector": {"device_id": "hand", "channels": ["gripper"]},
            },
            "steps": [{"capability": "joint_point", "target_rad": [0, 0], "speed_scale": 0.05}],
        }
        self.filename = self.directory / "device.json"
        self.save()

    def save(self):
        self.filename.write_text(json.dumps(self.plan), encoding="utf-8")

    def cli(self, operation, *flags):
        return subprocess.run(
            [CLI, operation, str(self.filename), *flags], capture_output=True, text=True, timeout=10
        )

    def test_capabilities_offline_even_when_another_owner_holds_the_lease(self):
        with DeviceClient(CLI, self.filename, allow_motion=False) as client:
            result = self.cli("capabilities")
            self.assertEqual(result.returncode, 0, result.stderr)
            supported = json.loads(result.stdout)
            self.assertEqual(supported, client.capabilities())
            self.assertTrue(supported["arm"]["disable"])
            self.assertTrue(supported["effector"]["enable"])
            self.assertTrue(client.state()["effector_enabled"])

    def test_existing_enable_permission_does_not_grant_new_torque_operations(self):
        with DeviceClient(CLI, self.filename, allow_enable=True) as client:
            self.assertEqual(client.enable_arm()["status"], "succeeded")
            before = client.state()
            for operation in (client.disable_arm, client.enable_effector, client.disable_effector):
                with self.assertRaisesRegex(DeviceCommandError, "requires"):
                    operation()
            state = client.state()
            self.assertEqual(
                (state["enabled"], state["effector_enabled"]),
                (before["enabled"], before["effector_enabled"]),
            )
            client.jog_arm(0, 0.1)

    def test_each_torque_permission_is_independent(self):
        cases = (
            ("allow_enable", "enable_arm"),
            ("allow_disable", "disable_arm"),
            ("allow_effector_enable", "enable_effector"),
            ("allow_effector_disable", "disable_effector"),
        )
        for permission, selected in cases:
            with self.subTest(permission=permission):
                with DeviceClient(CLI, self.filename, **{permission: True}) as client:
                    for _, operation in cases:
                        if operation == selected:
                            self.assertEqual(getattr(client, operation)()["status"], "succeeded")
                        else:
                            with self.assertRaisesRegex(DeviceCommandError, "requires"):
                                getattr(client, operation)()

    def test_read_only_rejects_holds_jogs_and_all_torque_operations(self):
        with DeviceClient(CLI, self.filename, allow_motion=False) as client:
            for operation in (
                client.enable_arm,
                client.disable_arm,
                client.enable_effector,
                client.disable_effector,
                client.hold_arm,
                client.hold_effector,
                lambda: client.jog_arm(0, 0.01),
            ):
                with self.assertRaisesRegex(DeviceCommandError, "forbidden"):
                    operation()
            self.assertEqual(client.state()["joints_rad"], [0, 0])
        for permission in (
            "allow_enable",
            "allow_disable",
            "allow_effector_enable",
            "allow_effector_disable",
        ):
            with self.assertRaises(ValueError):
                DeviceClient(CLI, self.filename, allow_motion=False, **{permission: True})
        for flag in (
            "--enable",
            "--allow-disable",
            "--allow-effector-enable",
            "--allow-effector-disable",
        ):
            result = self.cli("serve", "--read-only", flag)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("excludes torque permissions", result.stderr)

    def test_holds_preserve_torque_and_disabled_devices_stay_disabled(self):
        with DeviceClient(
            CLI, self.filename, allow_disable=True, allow_effector_disable=True
        ) as client:
            client.hold_arm()
            client.hold_effector()
            self.assertTrue(client.state()["enabled"])
            self.assertTrue(client.state()["effector_enabled"])
            client.disable_effector()
            client.disable_arm()
            for operation in (client.hold_arm, client.hold_effector):
                with self.assertRaisesRegex(DeviceCommandError, "disabled"):
                    operation()
            self.assertFalse(client.state()["enabled"])
            self.assertFalse(client.state()["effector_enabled"])

    def test_hold_remains_available_after_deadline_without_clearing_the_latch(self):
        with DeviceClient(
            CLI,
            self.filename,
            allow_enable=True,
            allow_disable=True,
            allow_effector_enable=True,
            allow_effector_disable=True,
        ) as client:
            # The arm stays still; the tool target is scheduled after this request expires.
            # This exercises the service deadline latch without a timing race on a short point move.
            with self.assertRaisesRegex(DeviceCommandError, "deadline exceeded"):
                client.move_arm_trajectory(
                    [0, 0.01], [[0, 0], [0, 0]], effector_schedule=[(0.5, [0.2], 0.1)], timeout=0.2
                )
            self.assertEqual(client.hold_arm()["status"], "succeeded")
            self.assertEqual(client.hold_effector()["status"], "succeeded")
            for operation in (
                client.enable_arm,
                client.disable_arm,
                client.enable_effector,
                client.disable_effector,
                lambda: client.jog_arm(0, 0.01),
            ):
                with self.assertRaisesRegex(DeviceCommandError, "deadline latched"):
                    operation()
            state = client.state()
            self.assertTrue(state["enabled"])
            self.assertTrue(state["effector_enabled"])
            self.assertEqual(state["joints_rad"], [0, 0])
            self.assertEqual(state["effector_positions"], [1])  # A Mock hand rests open.

    def test_read_only_permission_cannot_be_overridden_in_a_request(self):
        with DeviceClient(CLI, self.filename, allow_motion=False) as client:
            for operation in (
                "enable",
                "disable",
                "effector_enable",
                "effector_disable",
                "arm_hold",
                "effector_hold",
            ):
                with self.assertRaisesRegex(DeviceCommandError, "forbidden"):
                    client.call(operation)
                with self.assertRaisesRegex(DeviceCommandError, "unknown/duplicate request field"):
                    client.call(operation, allow_motion=True, allow_enable=True)
            self.assertEqual(client.state()["joints_rad"], [0, 0])
            self.assertTrue(client.state()["enabled"])

    def test_jog_obeys_joint_limits_and_invalid_requests_leave_motion_usable(self):
        with DeviceClient(CLI, self.filename) as client:
            for fields in (
                dict(joint_index=2, delta_rad=0.1, speed_scale=0.05),
                dict(joint_index=-1, delta_rad=0.1, speed_scale=0.05),
                dict(joint_index=0, delta_rad=2, speed_scale=0.05),
                dict(joint_index=0, delta_rad=0.1, speed_scale=0),
                dict(joint_index=0, delta_rad=0.1, speed_scale=2),
            ):
                with self.assertRaises(DeviceCommandError):
                    client.call("arm_jog", **fields)
            for index in (-1, 0.5, True):
                with self.assertRaises(ValueError):
                    client.jog_arm(index, 0.1)
            with self.assertRaises(ValueError):
                client.jog_arm(0, float("nan"))
            self.assertEqual(client.state()["joints_rad"], [0, 0])
            client.move_arm([0.5, -0.5], 0.05)
            client.jog_arm(0, 0.2)
            self.assertEqual(client.state()["joints_rad"], [0.7, -0.5])
            with self.assertRaises(DeviceCommandError):
                client.jog_arm(0, 0.5)
            self.assertEqual(client.state()["joints_rad"], [0.7, -0.5])

    def test_open_loop_feedback_still_has_no_positions_or_verified_hold(self):
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            state = client.state()
            self.assertEqual(
                (state["effector_feedback"], state["effector_positions"]), ("none", [])
            )
            self.assertFalse(client.capabilities()["effector"]["verified_hold"])
            with self.assertRaisesRegex(DeviceCommandError, "unsupported"):
                client.hold_effector()
            result = client.move_effector([0.5], 0.05, allow_unverified=True)
            self.assertEqual(result["status"], "unverified")

    def test_arm_only_capabilities_have_no_effector(self):
        del self.plan["device"]["effector"]
        self.save()
        with DeviceClient(CLI, self.filename, allow_effector_enable=True) as client:
            self.assertIsNone(client.capabilities()["effector"])
            with self.assertRaisesRegex(DeviceCommandError, "no end effector"):
                client.enable_effector()
            client.jog_arm(0, 0.1)

    def test_torque_flags_cannot_silently_apply_to_run_or_inspect(self):
        for operation in ("run", "inspect", "capabilities"):
            for flag in ("--allow-disable", "--allow-effector-enable", "--allow-effector-disable"):
                self.assertNotEqual(self.cli(operation, flag).returncode, 0)

    def test_operator_commands_request_only_the_named_torque_permission(self):
        cases = {
            "enable": ("enable_arm", "allow_enable"),
            "disable": ("disable_arm", "allow_disable"),
            "hand-enable": ("enable_effector", "allow_effector_enable"),
            "hand-disable": ("disable_effector", "allow_effector_disable"),
            "hold": ("hold_arm", None),
            "hand-hold": ("hold_effector", None),
        }
        for operation, (method, permission) in cases.items():
            with self.subTest(operation=operation):
                with (
                    patch.object(
                        sys, "argv", ["manipulation", "--prefix", str(self.directory), operation]
                    ),
                    patch.object(operator_cli, "Context") as context_type,
                    patch("spacemit_manipulation.client.DeviceClient") as client_type,
                    patch("builtins.print"),
                ):
                    context = context_type.return_value
                    context.binary.return_value = CLI
                    context.plan.return_value = self.filename
                    operator_cli.main()
                    expected = {permission: True} if permission else {}
                    client_type.assert_called_once_with(CLI, self.filename, **expected)
                    getattr(
                        client_type.return_value.__enter__.return_value, method
                    ).assert_called_once_with()

    def test_follow_sends_each_target_without_waiting_and_keeps_every_check(self):
        with DeviceClient(CLI, self.filename, allow_effector_disable=True) as client:
            for step in range(1, 6):
                result = client.follow_arm([0.1 * step, -0.1 * step], 0.5)
                self.assertEqual(result["status"], "succeeded")
                self.assertTrue(result["detail"].startswith("sent"), result)
                self.assertTrue(
                    client.follow_effector([1 - 0.1 * step])["detail"].startswith("sent")
                )
            self.assertAlmostEqual(client.state()["effector_positions"][0], 0.5)
            self.assertEqual(client.hold_arm()["status"], "succeeded")
            self.assertEqual(
                client.move_arm([0, 0], 0.5)["status"], "succeeded"
            )  # A followed target leaves no latch.
            for bad in ([2.0, 0.0], [0.0]):
                with self.assertRaises(DeviceCommandError):
                    client.follow_arm(bad, 0.5)
            for bad in ([1.5], [0.5, 0.5]):
                with self.assertRaises(DeviceCommandError):
                    client.follow_effector(bad)
            self.assertEqual(client.state()["joints_rad"], [0, 0])
            client.disable_effector()
            with self.assertRaises(DeviceCommandError):
                client.follow_effector([0.5])

    def test_follow_without_position_feedback_is_unverified(self):
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError):
                client.follow_effector([0.5])
            self.assertEqual(
                client.follow_effector([0.5], allow_unverified=True)["status"], "unverified"
            )

    def test_read_only_session_refuses_follow(self):
        with DeviceClient(CLI, self.filename, allow_motion=False) as client:
            for operation in (
                lambda: client.follow_arm([0.1, 0.1], 0.5),
                lambda: client.follow_effector([0.5]),
            ):
                with self.assertRaises(DeviceCommandError):
                    operation()

    def test_pose_move_refuses_a_point_move_longer_than_the_timeout(self):
        state = dict(joints_rad=[0.0, 0.0], controller_limits=[dict(max_velocity_rad_s=1.0)] * 2)
        operator_cli.check_duration(state, [0.4, -0.2], 0.05, 10)  # At least 8 s.
        with self.assertRaisesRegex(ValueError, "nothing moved"):
            operator_cli.check_duration(state, [0.6, 0.0], 0.05, 10)  # At least 12 s.
        operator_cli.check_duration(
            dict(joints_rad=[0.0, 0.0]), [3.0, 3.0], 0.05, 10
        )  # No velocity reported.

    @unittest.skipUnless(os.environ.get("MANIPULATION_TEST_NERO") == "1", "Nero disabled")
    def test_nero_revo2_reject_new_unsupported_controls_without_vendor_writes(self):
        sdk = self.directory / "sdk"
        shutil.copytree(ROOT / "tests/fake_sdk", sdk)
        (sdk / "UPSTREAM_COMMIT").write_text(
            "e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b\n", encoding="utf-8"
        )
        (sdk / "pyAgxArm/recovery.json").write_text('{"mode":"ok"}', encoding="utf-8")
        channel = f"t{os.getpid() % 0x10000:04x}_recover"
        device = self.plan["device"]
        device["driver"] = "agx_nero_revo2"
        device["arm"]["resource_id"] = "agx_" + channel
        device["arm"]["joints"] = [
            dict(name=f"joint{i}", minimum_rad=-1, maximum_rad=1) for i in range(1, 8)
        ]
        device["effector"].update(
            channels=[
                "thumb_tip",
                "thumb_base",
                "index_finger",
                "middle_finger",
                "ring_finger",
                "pinky_finger",
            ],
            feedback="none",
        )
        device["options"] = dict(
            worker=str(ROOT / "drivers/common/agx_session/agx_worker.py"),
            sdk_root=str(sdk),
            python=sys.executable,
            channel=channel,
        )
        self.plan["steps"][0]["target_rad"] = [0] * 7
        self.save()
        trace = sdk / "pyAgxArm/recovery-events.jsonl"
        with DeviceClient(
            CLI,
            self.filename,
            allow_disable=True,
            allow_effector_enable=True,
            allow_effector_disable=True,
        ) as client:
            before = trace.read_bytes() if trace.exists() else b""
            capabilities = client.capabilities()
            self.assertFalse(capabilities["arm"]["disable"])
            self.assertFalse(capabilities["effector"]["enable"])
            self.assertFalse(capabilities["effector"]["disable"])
            self.assertIsNone(client.state()["effector_enabled"])
            for operation in (
                client.disable_arm,
                client.enable_effector,
                client.disable_effector,
                client.hold_effector,
            ):
                with self.assertRaisesRegex(DeviceCommandError, "unsupported"):
                    operation()
            self.assertEqual(trace.read_bytes() if trace.exists() else b"", before)
            client.move_arm(
                [0.01] * 7, 0.05
            )  # Unsupported operations did not latch the established control path.


if __name__ == "__main__":
    unittest.main()
