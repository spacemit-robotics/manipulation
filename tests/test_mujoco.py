#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Headless real-physics integration through the unchanged CLI and public Python client."""

import copy
import json
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
CLI = os.environ["MANIPULATION_DEVICE"]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402


class MujocoTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="mujoco-cli-contract-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        shutil.copyfile(ROOT / "tests/fixtures/mujoco/scene.xml", self.directory / "scene.xml")
        self.filename = self.directory / "device.json"
        self.mapping_file = self.directory / "mapping.json"
        self.mapping = dict(
            schema_version=1,
            scene="scene.xml",
            initial_keyframe="start",
            arm=[
                dict(joint="second_joint", actuator="second_servo", max_velocity_rad_s=0.6),
                dict(joint="first_joint", actuator="first_servo", max_velocity_rad_s=0.6),
            ],
            effector=[
                dict(
                    channel="gripper",
                    joint="finger_joint",
                    actuator="finger_servo",
                    joint_closed=1,
                    joint_open=0,
                    control_closed=1,
                    control_open=0,
                )
            ],
            arm_moving_threshold_rad_s=0.004,
            effector_moving_threshold=0.004,
            hold_timeout_s=3,
        )
        self.plan = dict(
            schema_version=3,
            device=dict(
                arm=dict(
                    driver="mujoco",
                    device_id="arm",
                    resource_id="mujoco_test_world",
                    joints=[
                        dict(name="second_joint", minimum_rad=-1, maximum_rad=1),
                        dict(name="first_joint", minimum_rad=-1, maximum_rad=1),
                    ],
                    timeout_s=5,
                    options=dict(mapping_file="mapping.json"),
                ),
                effector=dict(
                    driver="mujoco",
                    device_id="tool",
                    resource_id="mujoco_test_world",
                    channels=["gripper"],
                    timeout_s=5,
                    options=dict(mapping_file="mapping.json"),
                ),
            ),
            steps=[],
        )
        self.environment = dict(os.environ, MANIPULATION_LOCK_DIRECTORY=str(self.directory))

    def save(self):
        self.filename.write_text(json.dumps(self.plan), encoding="utf-8")
        self.mapping_file.write_text(json.dumps(self.mapping), encoding="utf-8")

    def call(self, operation):
        self.save()
        return subprocess.run(
            [CLI, operation, str(self.filename)],
            cwd="/",
            env=self.environment,
            text=True,
            capture_output=True,
            timeout=20,
        )

    def test_normal_sequence_and_configuration_relative_paths(self):
        self.plan["steps"] = [
            dict(capability="joint_point", target_rad=[0.1, -0.08], speed_scale=0.8),
            dict(capability="effector_position", positions=[0.3], duration_s=0.15),
        ]
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status":"succeeded"', result.stdout)

    def test_public_client_reads_and_moves_real_feedback(self):
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(len(client.state()["joints_rad"]), 2)
            result = client.move_arm([0.12, -0.05], speed_scale=0.8)
            self.assertEqual(result["status"], "succeeded")
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0.12, delta=0.01)
            result = client.move_effector([0.25], duration_s=0.15)
            self.assertEqual(result["status"], "succeeded")
            self.assertAlmostEqual(client.state()["effector_positions"][0], 0.25, delta=0.02)

    def test_small_slow_effector_move_waits_until_dispatch_finishes(self):
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            before = client.state()["effector_positions"][0]
            started = time.monotonic()
            result = client.move_effector([before + 0.001], duration_s=0.4)
            self.assertEqual(result["status"], "succeeded")
            self.assertGreaterEqual(time.monotonic() - started, 0.38)

    def test_synchronized_point_is_explicit_and_preserves_default_independent_rates(self):
        for synchronized in (False, True):
            with self.subTest(synchronized=synchronized):
                if synchronized:
                    self.mapping["synchronized_joint_point"] = True
                self.save()
                with DeviceClient(CLI, self.filename) as client:
                    client.follow_arm([0.3, -0.15], speed_scale=0.5)
                    time.sleep(0.25)
                    joints = client.state()["joints_rad"]
                    self.assertGreater(joints[0], 0.03)
                    if synchronized:
                        self.assertAlmostEqual(joints[0], -2 * joints[1], delta=0.008)
                    else:
                        self.assertAlmostEqual(joints[0], -joints[1], delta=0.008)
                    client.hold_arm()

    def test_offline_factory_does_not_load_or_step_scene(self):
        self.mapping["scene"] = "missing-scene.xml"
        self.assertEqual(self.call("validate").returncode, 0)
        self.assertEqual(self.call("capabilities").returncode, 0)
        self.assertNotEqual(self.call("inspect").returncode, 0)
        self.mapping["scene"] = "scene.xml"
        self.assertEqual(self.call("inspect").returncode, 0, "failed connect leaked resource")

    def test_arm_only_and_separate_mock_effector(self):
        del self.plan["device"]["effector"]
        result = self.call("inspect")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(json.loads(result.stdout)["joints_rad"]), 2)
        self.plan["device"]["effector"] = dict(
            driver="mock", device_id="tool", resource_id="separate_mock", channels=["finger"]
        )
        self.assertEqual(self.call("inspect").returncode, 0)

    def test_independent_tool_with_mock_arm(self):
        self.plan["device"]["arm"] = dict(
            driver="mock",
            device_id="arm",
            resource_id="separate_mock",
            joints=[dict(name="joint", minimum_rad=-1, maximum_rad=1)],
        )
        self.plan["steps"] = [
            dict(capability="effector_position", positions=[0.3], duration_s=0.15)
        ]
        result = self.call("run")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_mapping_validation(self):
        original = copy.deepcopy(self.mapping)
        invalid = [
            lambda mapping: mapping.update(schema_version=2),
            lambda mapping: mapping.update(unknown="field"),
            lambda mapping: mapping.update(arm_moving_threshold_rad_s=0),
            lambda mapping: mapping["arm"].reverse(),
            lambda mapping: mapping["arm"][0].update(max_velocity_rad_s=float("nan")),
            lambda mapping: mapping["arm"][0].update(actuator="first_servo"),
            lambda mapping: mapping["effector"][0].update(joint_open=1),
            lambda mapping: mapping["effector"][0].update(channel="wrong_channel"),
        ]
        for index, change in enumerate(invalid):
            with self.subTest(case=index):
                self.mapping = copy.deepcopy(original)
                change(self.mapping)
                self.assertNotEqual(self.call("validate").returncode, 0)

    def test_duplicate_keys_rejected(self):
        self.save()
        self.mapping_file.write_text(
            "schema_version: 1\nschema_version: 1\nscene: scene.xml\n", encoding="utf-8"
        )
        result = subprocess.run(
            [CLI, "validate", str(self.filename)], capture_output=True, text=True, timeout=10
        )
        self.assertNotEqual(result.returncode, 0)

    def test_shared_roles_require_same_mapping(self):
        self.save()
        shutil.copyfile(self.mapping_file, self.directory / "different.json")
        self.plan["device"]["effector"]["options"]["mapping_file"] = "different.json"
        self.assertNotEqual(self.call("validate").returncode, 0)

    def test_model_mismatch_rejected_before_runtime(self):
        original = copy.deepcopy(self.mapping)
        invalid = [
            lambda mapping: mapping.update(initial_keyframe="missing"),
            lambda mapping: mapping["arm"][0].update(actuator="missing_servo"),
            lambda mapping: mapping["effector"][0].update(joint="missing_joint"),
            lambda mapping: mapping["effector"][0].update(control_closed=2),
        ]
        for index, change in enumerate(invalid):
            with self.subTest(case=index):
                self.mapping = copy.deepcopy(original)
                change(self.mapping)
                self.assertNotEqual(self.call("inspect").returncode, 0)
        self.mapping = original
        self.assertEqual(self.call("inspect").returncode, 0)

    def test_wrong_actuator_transmission_rejected(self):
        self.mapping["arm"][0]["actuator"] = "first_servo"
        self.mapping["arm"][1]["actuator"] = "second_servo"
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("mapped joint", result.stderr)

    def test_nonunit_arm_transmission_rejected(self):
        scene = self.directory / "scene.xml"
        text = scene.read_text(encoding="utf-8")
        scene.write_text(text.replace('name="first_servo"', 'name="first_servo" gear="2"'))
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unit position servo", result.stderr)

    def test_effector_transmission_must_drive_mapped_joint(self):
        self.mapping["effector"][0]["joint"] = "passive_joint"
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("does not control its mapped joint", result.stderr)

    def test_effector_tendon_cannot_also_control_arm(self):
        scene = self.directory / "scene.xml"
        text = scene.read_text(encoding="utf-8")
        text = text.replace(
            '<joint joint="finger_joint" coef="1"/>',
            '<joint joint="finger_joint" coef="1"/><joint joint="first_joint" coef="1"/>',
        )
        scene.write_text(text, encoding="utf-8")
        result = self.call("inspect")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("also controls an arm joint", result.stderr)

    def test_disabled_scene_actuation_is_not_reported_enabled(self):
        scene = self.directory / "scene.xml"
        text = scene.read_text(encoding="utf-8")
        text = text.replace(
            '<option timestep="0.002" gravity="0 0 0"/>',
            '<option timestep="0.002" gravity="0 0 0"><flag actuation="disable"/></option>',
        )
        scene.write_text(text, encoding="utf-8")
        self.assertNotEqual(self.call("inspect").returncode, 0)

    def test_mismatched_initial_controls_rejected_without_startup_motion(self):
        scene = self.directory / "scene.xml"
        original = scene.read_text(encoding="utf-8")
        for controls in ("0.3 0 0.5", "0 0 0.8"):
            with self.subTest(controls=controls):
                text = original.replace('ctrl="0 0 0.5"', f'ctrl="{controls}"')
                scene.write_text(text, encoding="utf-8")
                result = self.call("inspect")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("unrequested motion", result.stderr)

    def test_unsupported_trajectory_rejected_offline(self):
        self.plan["steps"] = [
            dict(capability="joint_trajectory", times_s=[0, 1], positions_rad=[[0, 0], [0.1, 0.1]])
        ]
        result = self.call("validate")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported", result.stderr)

    def test_stale_feedback_rejected_and_lease_released(self):
        self.plan["device"]["arm"]["maximum_age_s"] = 1e-12
        self.assertNotEqual(self.call("inspect").returncode, 0)
        self.plan["device"]["arm"]["maximum_age_s"] = 0.2
        self.assertEqual(self.call("inspect").returncode, 0)

    def test_hold_without_fresh_stop_evidence_is_unknown(self):
        self.mapping["hold_timeout_s"] = 1e-6
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError) as caught:
                client.hold_arm()
            self.assertEqual(caught.exception.result["status"], "stop_unknown")
            self.assertTrue(client.state()["latched"])

    def test_sequence_failure_reports_its_cause(self):
        self.plan["device"]["arm"]["timeout_s"] = 0.01
        self.plan["steps"] = [
            dict(capability="joint_point", target_rad=[0.5, -0.5], speed_scale=0.1)
        ]
        result = self.call("run")
        self.assertNotEqual(result.returncode, 0)
        final = json.loads(result.stdout.splitlines()[-1])
        self.assertNotEqual(final["status"], "succeeded")
        self.assertTrue(final["detail"], "sequence errors must retain the command failure cause")

    def test_scheduler_failure_does_not_publish_reset_or_fake_samples(self):
        self.save()
        with DeviceClient(CLI, self.filename) as client:
            # Complete a physics tick so the fault carries its resource counters.
            time.sleep(0.02)
            client.process.send_signal(signal.SIGSTOP)
            try:
                time.sleep(0.25)
            finally:
                client.process.send_signal(signal.SIGCONT)
            time.sleep(0.03)
            with self.assertRaises(DeviceCommandError) as caught:
                client.state()
            detail = str(caught.exception)
            self.assertIn("real-time physics scheduling overrun", detail)
            for field in ("previous_cpu_ms", "previous_minor_faults", "previous_major_faults"):
                value = r"[0-9]+(?:\.[0-9]+)?" if sys.platform == "linux" else "unavailable"
                self.assertRegex(detail, rf"{field}={value}(?![\w.])")
            with self.assertRaises(DeviceCommandError):
                client.move_arm([0.1, 0.1], speed_scale=0.5)
        # Recovery is explicit creation of a new simulation session, not an automatic reset.
        with DeviceClient(CLI, self.filename) as client:
            self.assertEqual(client.state()["joints_rad"], [0, 0])


if __name__ == "__main__":
    unittest.main()
