#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Independent role configuration through the normal CLI, fake CAN SDK and PTY serial bus."""

import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLI = os.environ["MANIPULATION_DEVICE"]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient  # noqa: E402


def mock_arm():
    """Small independently connected arm for testing another vendor's end effector."""
    return dict(
        driver="mock",
        device_id="mock_arm",
        resource_id="composition_mock_arm",
        joints=[dict(name="joint", minimum_rad=-1, maximum_rad=1)],
    )


class CompositionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="composition-plan-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.filename = self.directory / "device.json"
        self.plan = dict(
            schema_version=3,
            device=dict(
                arm=mock_arm(),
                effector=dict(
                    driver="mock",
                    device_id="mock_hand",
                    resource_id="composition_mock_hand",
                    channels=["finger"],
                    feedback="position",
                ),
            ),
            steps=[],
        )

    def save(self):
        self.filename.write_text(json.dumps(self.plan), encoding="utf-8")

    def call(self, command):
        self.save()
        return subprocess.run(
            [CLI, command, str(self.filename)], capture_output=True, text=True, timeout=10
        )

    def test_mock_roles_run_and_describe_offline(self):
        self.plan["steps"] = [
            dict(capability="joint_point", target_rad=[0.1], speed_scale=0.05),
            dict(capability="effector_position", positions=[0.2], duration_s=0.01),
        ]
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status":"succeeded"', result.stdout)

    def test_role_options_and_missing_resource_fail_closed(self):
        invalid = [
            ("arm", "driver", "unregistered"),
            ("effector", "driver", "unregistered"),
            ("effector", "resource_id", ""),
            ("effector", "resource_id", "../escape"),
            ("arm", "options", {"unknown": "value"}),
        ]
        original = copy.deepcopy(self.plan)
        for role, name, value in invalid:
            with self.subTest(role=role, name=name, value=value):
                self.plan = copy.deepcopy(original)
                self.plan["device"][role][name] = value
                self.assertNotEqual(self.call("validate").returncode, 0)

    def test_legacy_and_independent_selection_cannot_mix(self):
        for field, value in (("driver", "mock_device"), ("options", {})):
            with self.subTest(field=field):
                self.plan["device"][field] = value
                self.assertNotEqual(self.call("validate").returncode, 0)
                del self.plan["device"][field]

    def test_independent_arm_without_effector(self):
        del self.plan["device"]["effector"]
        result = self.call("inspect")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["joints_rad"], [0])

    def nero_fixture(self):
        if os.environ.get("MANIPULATION_TEST_NERO") != "1":
            self.skipTest("Nero driver not built")
        import test_nero

        fixture = test_nero.NeroIntegrationTests()
        fixture.addCleanup = self.addCleanup
        fixture.setUp()
        fixture.channel("can_lag")
        return fixture

    def so101_fixture(self):
        if os.environ.get("MANIPULATION_TEST_SO101") != "1":
            self.skipTest("SO101 driver not built")
        import test_so101

        fixture = test_so101.So101Tests()
        fixture.addCleanup = self.addCleanup
        fixture.setUp()
        fixture.gripper()
        return fixture

    def revo2(self, fixture):
        return dict(
            driver="revo2",
            device_id="tool",
            resource_id=fixture.plan["device"]["arm"]["resource_id"],
            channels=[
                "thumb_tip",
                "thumb_base",
                "index_finger",
                "middle_finger",
                "ring_finger",
                "pinky_finger",
            ],
            feedback="none",
            options=dict(fixture.options),
        )

    def test_mock_arm_with_independent_revo2_needs_no_arm_feedback(self):
        fixture = self.nero_fixture()
        self.plan["device"]["effector"] = self.revo2(fixture)
        # The vendor receiver provides invalid arm data; an independent hand must not use it.
        sdk = fixture.sdk / "pyAgxArm/__init__.py"
        source = sdk.read_text(encoding="utf-8")
        source = source.replace('if mode == "part":', 'if mode in ("part", "lag"):')
        sdk.write_text(source, encoding="utf-8")
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(client.state()["joints_rad"], [0])
            result = client.move_effector([0.5] * 6, duration_s=0.01, allow_unverified=True)
            self.assertEqual(result["status"], "unverified")
        self.assertEqual(fixture.events().count("connect"), 1)
        self.assertFalse({"enable", "move_j", "move_js"} & set(fixture.events()))

    def test_same_resource_nero_revo2_share_one_connection(self):
        fixture = self.nero_fixture()
        arm = copy.deepcopy(fixture.plan["device"]["arm"])
        arm.update(driver="nero", options=dict(fixture.options))
        self.plan["device"] = dict(arm=arm, effector=self.revo2(fixture))
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(len(client.state()["joints_rad"]), 7)
            self.assertEqual(
                client.move_effector([0.5] * 6, 0.01, allow_unverified=True)["status"], "unverified"
            )
        self.assertEqual(fixture.events().count("connect"), 1)

    def test_shared_agx_conflicting_options_rejected_offline(self):
        fixture = self.nero_fixture()
        arm = copy.deepcopy(fixture.plan["device"]["arm"])
        arm.update(driver="nero", options=dict(fixture.options))
        self.plan["device"] = dict(arm=arm, effector=self.revo2(fixture))
        self.plan["device"]["effector"]["options"]["python"] = "/different/python"
        result = self.call("validate")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("identical transport options", result.stderr)
        self.assertEqual(fixture.events(), [])

    def test_nero_arm_and_mock_effector(self):
        fixture = self.nero_fixture()
        arm = copy.deepcopy(fixture.plan["device"]["arm"])
        arm.update(driver="nero", options=dict(fixture.options))
        self.plan["device"]["arm"] = arm
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(len(client.state()["joints_rad"]), 7)
            self.assertEqual(client.state()["effector_positions"], [1])
        self.assertNotIn("init_effector", fixture.events())

    def test_mock_arm_and_so101_gripper_only_access_gripper(self):
        fixture = self.so101_fixture()
        effector = copy.deepcopy(fixture.plan["device"]["effector"])
        effector.update(
            driver="so101_gripper",
            resource_id="composition_so101_gripper",
            options=dict(
                port=fixture.bus.port,
                gripper_calibration=str(fixture.directory / "gripper.json"),
            ),
        )
        self.plan["device"]["effector"] = effector
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(client.state()["joints_rad"], [0])
            self.assertEqual(len(client.state()["effector_positions"]), 1)
        self.assertTrue(fixture.bus.requests)
        self.assertEqual({motor_id for motor_id, _, _ in fixture.bus.requests}, {6})
        self.assertEqual(fixture.bus.writes, [])

    def test_shared_so101_roles_keep_one_serial_connection(self):
        fixture = self.so101_fixture()
        arm = copy.deepcopy(fixture.plan["device"]["arm"])
        arm.update(
            driver="so101",
            options=dict(
                port=fixture.bus.port, arm_calibration=str(fixture.directory / "arm.json")
            ),
        )
        effector = copy.deepcopy(fixture.plan["device"]["effector"])
        effector.update(
            driver="so101_gripper",
            resource_id=arm["resource_id"],
            options=dict(
                port=fixture.bus.port,
                gripper_calibration=str(fixture.directory / "gripper.json"),
            ),
        )
        self.plan["device"] = dict(arm=arm, effector=effector)
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(len(client.state()["joints_rad"]), 5)
            self.assertEqual(len(client.state()["effector_positions"]), 1)
        self.assertEqual({motor_id for motor_id, _, _ in fixture.bus.requests}, set(range(1, 7)))
        self.assertEqual(fixture.bus.writes, [])

    def test_shared_so101_custom_ids_and_collision_validation(self):
        fixture = self.so101_fixture()
        fixture.arm_calibration["wrist_roll"]["id"] = 6
        fixture.gripper_calibration["motor_id"] = 7
        fixture.save()
        arm = copy.deepcopy(fixture.plan["device"]["arm"])
        arm_options = dict(
            port=fixture.bus.port,
            arm_ids="1,2,3,4,6",
            arm_calibration=str(fixture.directory / "arm.json"),
        )
        effector = copy.deepcopy(fixture.plan["device"]["effector"])
        effector_options = dict(
            port=fixture.bus.port,
            gripper_id="7",
            gripper_calibration=str(fixture.directory / "gripper.json"),
        )
        for composed in (False, True):
            with self.subTest(composed=composed):
                if composed:
                    self.plan["device"] = dict(
                        arm=dict(arm, driver="so101", options=arm_options),
                        effector=dict(
                            effector,
                            driver="so101_gripper",
                            resource_id=arm["resource_id"],
                            options=effector_options,
                        ),
                    )
                else:
                    self.plan["device"] = dict(
                        driver="so101",
                        arm=arm,
                        effector=effector,
                        options=dict(arm_options, **effector_options),
                    )
                result = self.call("validate")
                self.assertEqual(result.returncode, 0, result.stderr)

                fixture.gripper_calibration["motor_id"] = 6
                fixture.save()
                if composed:
                    self.plan["device"]["effector"]["options"] = dict(
                        effector_options, gripper_id="6"
                    )
                else:
                    self.plan["device"]["options"]["gripper_id"] = "6"
                result = self.call("validate")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("arm and gripper IDs overlap", result.stderr)
                fixture.gripper_calibration["motor_id"] = 7
                fixture.save()
        self.assertEqual(fixture.bus.requests, [])

        # With no gripper on this transport, servo 6 is an ordinary arm ID.
        self.plan["device"] = dict(arm=dict(arm, driver="so101", options=arm_options))
        result = self.call("validate")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(fixture.bus.requests, [])

    def test_so101_arm_and_revo2_use_independent_transports(self):
        serial = self.so101_fixture()
        vendor = self.nero_fixture()
        arm = copy.deepcopy(serial.plan["device"]["arm"])
        arm.update(
            driver="so101",
            options=dict(port=serial.bus.port, arm_calibration=str(serial.directory / "arm.json")),
        )
        self.plan["device"] = dict(arm=arm, effector=self.revo2(vendor))
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            state = client.state()
            self.assertEqual(len(state["joints_rad"]), 5)
            self.assertEqual(state["effector_feedback"], "none")
            self.assertEqual(
                client.move_effector([0.5] * 6, 0.01, allow_unverified=True)["status"], "unverified"
            )
        self.assertEqual({motor_id for motor_id, _, _ in serial.bus.requests}, set(range(1, 6)))
        self.assertEqual(serial.bus.writes, [])
        self.assertEqual(vendor.events().count("connect"), 1)

    def test_nero_arm_and_so101_gripper_use_independent_transports(self):
        serial = self.so101_fixture()
        vendor = self.nero_fixture()
        arm = copy.deepcopy(vendor.plan["device"]["arm"])
        arm.update(driver="nero", options=dict(vendor.options))
        effector = copy.deepcopy(serial.plan["device"]["effector"])
        effector.update(
            driver="so101_gripper",
            resource_id="composition_serial_gripper",
            options=dict(
                port=serial.bus.port,
                gripper_calibration=str(serial.directory / "gripper.json"),
            ),
        )
        self.plan["device"] = dict(arm=arm, effector=effector)
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            state = client.state()
            self.assertEqual(len(state["joints_rad"]), 7)
            self.assertEqual(len(state["effector_positions"]), 1)
        self.assertEqual({motor_id for motor_id, _, _ in serial.bus.requests}, {6})
        self.assertNotIn("init_effector", vendor.events())

    def test_same_resource_requires_a_registered_shared_pair(self):
        fixture = self.nero_fixture()
        self.plan["device"]["effector"] = self.revo2(fixture)
        self.plan["device"]["arm"]["resource_id"] = self.plan["device"]["effector"]["resource_id"]
        result = self.call("validate")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("cannot share", result.stderr)
        self.assertEqual(fixture.events(), [])


if __name__ == "__main__":
    unittest.main()
