# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Registered capability dispatch through the local process; only Mock devices are used."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402

CLI = os.environ["MANIPULATION_DEVICE"]


class CapabilityClientTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="capability-client-")
        self.addCleanup(temporary.cleanup)
        self.plan_path = Path(temporary.name) / "device.json"
        self.plan = {
            "schema_version": 3,
            "device": {
                "arm": {
                    "driver": "mock",
                    "device_id": "arm",
                    "resource_id": "capability_client_arm",
                    "joints": [{"name": "joint", "minimum_rad": -1, "maximum_rad": 1}],
                },
                "effector": {
                    "driver": "mock",
                    "device_id": "hand",
                    "resource_id": "capability_client_tool",
                    "channels": ["gripper"],
                },
            },
            "steps": [],
        }
        self.save()

    def save(self):
        self.plan_path.write_text(json.dumps(self.plan), encoding="utf-8")

    def test_catalogue_and_registered_motion(self):
        with DeviceClient(CLI, self.plan_path) as client:
            self.assertEqual(client.metadata["driver"], "composed")
            catalogue = {item["id"]: item for item in client.capabilities()["registered"]}
            for name in ("joint_point", "joint_jog", "effector_position"):
                self.assertTrue(catalogue[name]["supported"])
            result = client.execute_capability(
                "joint_point", {"target_rad": [0.1], "speed_scale": 0.05}
            )
            self.assertEqual(result["status"], "succeeded")
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0.1, places=3)
            client.execute_capability("effector_position", {"positions": [0.2], "duration_s": 0.1})
            self.assertAlmostEqual(client.state()["effector_positions"][0], 0.2, places=3)
            # Existing explicit methods continue to operate on the same composed session.
            client.move_arm([0.0], 0.05)
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0, places=3)

    def test_invalid_capabilities_never_write(self):
        with DeviceClient(CLI, self.plan_path) as client:
            for name, parameters in (
                ("unknown", {}),
                ("joint_point", {"target_rad": [5], "speed_scale": 0.05}),
                ("joint_point", {"target_rad": [0.2], "speed_scale": 0.05, "ignored": True}),
            ):
                with self.assertRaises(DeviceCommandError) as raised:
                    client.execute_capability(name, parameters)
                self.assertTrue(raised.exception.rejected)
                self.assertAlmostEqual(client.state()["joints_rad"][0], 0, places=3)
                self.assertFalse(client.state()["latched"])
            with self.assertRaises(ValueError):
                client.execute_capability("joint_point", {"capability": "arm_hold"})

    def test_read_only_and_pre_cancel_preserve_state(self):
        parameters = {"target_rad": [0.2], "speed_scale": 0.05}
        with DeviceClient(CLI, self.plan_path, allow_motion=False) as client:
            with self.assertRaises(DeviceCommandError):
                client.execute_capability("joint_point", parameters)
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0, places=3)
        with DeviceClient(CLI, self.plan_path) as client:
            with self.assertRaises(DeviceCommandError) as raised:
                client.execute_capability("joint_point", parameters, is_cancelled=lambda: True)
            self.assertEqual(raised.exception.result["status"], "cancelled")
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0, places=3)

    def test_open_loop_never_claims_measured_completion(self):
        self.plan["device"]["effector"]["feedback"] = "none"
        self.save()
        with DeviceClient(CLI, self.plan_path) as client:
            result = client.execute_capability(
                "effector_position",
                {"positions": [0.2], "duration_s": 0.1},
                allow_unverified=True,
            )
            self.assertEqual(result["status"], "unverified")
            self.assertFalse(result["physical_grip_verified"])

    def test_role_discovery_does_not_connect(self):
        result = subprocess.run([CLI, "driver-roles"], capture_output=True, text=True, check=True)
        self.assertIn("arm mock\n", result.stdout)
        self.assertIn("effector mock\n", result.stdout)
        result = subprocess.run(
            [CLI, "describe", str(self.plan_path)], capture_output=True, text=True, check=True
        )
        description = json.loads(result.stdout)
        self.assertEqual(description["driver"], "composed")
        self.assertEqual(description["arm_driver"], "mock")
        self.assertEqual(description["effector_driver"], "mock")


if __name__ == "__main__":
    unittest.main()
