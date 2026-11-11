#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline contracts for named simulation models and configurable mapping roles."""

import copy
import itertools
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CLI = os.environ["MANIPULATION_DEVICE"]
HAS_LOCAL = os.environ["MANIPULATION_TEST_MUJOCO"] == "1"


class SimulationDriverTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="simulation-drivers-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        self.mapping = {
            "schema_version": 1,
            "scene": "not-loaded-during-validation.xml",
            "arm": [
                {"joint": name + "_joint", "actuator": name, "max_velocity_rad_s": 1}
                for name in actuators
            ],
            "effector": [{
                "channel": "gripper", "joint": "gripper_right_driver_joint",
                "actuator": "gripper_fingers_actuator", "joint_closed": 0.8, "joint_open": 0,
                "control_closed": 255, "control_open": 0,
            }],
        }
        self.plan = {
            "schema_version": 3,
            "device": {
                "arm": {
                    "driver": "mujoco_ur5e", "device_id": "arm", "resource_id": "world",
                    "joints": [
                        {"name": item["joint"], "minimum_rad": -1, "maximum_rad": 1}
                        for item in self.mapping["arm"]
                    ],
                    "options": {"mapping_file": "mapping.json", "transport": "tcp"},
                },
                "effector": {
                    "driver": "mujoco_robotiq_2f85", "device_id": "tool", "resource_id": "world",
                    "channels": ["gripper"], "feedback": "position",
                    "options": {"mapping_file": "mapping.json", "transport": "tcp"},
                },
            },
            "steps": [],
        }

    def validate(self):
        (self.directory / "mapping.json").write_text(json.dumps(self.mapping), encoding="utf-8")
        plan = self.directory / "device.json"
        plan.write_text(json.dumps(self.plan), encoding="utf-8")
        return subprocess.run([CLI, "validate", str(plan)], capture_output=True, text=True, timeout=10)

    def test_registered_roles_identify_models_and_generic_mappings(self):
        result = subprocess.run([CLI, "driver-roles"], capture_output=True, text=True, check=True, timeout=10)
        roles = set(result.stdout.splitlines())
        self.assertTrue({"arm mujoco_ur5e", "arm mujoco_mapped_arm"} <= roles)
        self.assertTrue({"effector mujoco_robotiq_2f85", "effector mujoco_mapped_effector"} <= roles)
        self.assertNotIn("arm mujoco", roles)
        self.assertNotIn("effector mujoco", roles)
        self.assertNotIn("arm mujoco_robotiq_2f85", roles)
        self.assertNotIn("effector mujoco_ur5e", roles)

    def test_unnamed_mujoco_roles_are_rejected_before_loading_scene(self):
        original = copy.deepcopy(self.plan)
        for role in ("arm", "effector"):
            with self.subTest(role=role):
                self.plan = copy.deepcopy(original)
                self.plan["device"][role]["driver"] = "mujoco"
                result = self.validate()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(f"unknown {role} driver: mujoco", result.stderr)

    def test_shipped_model_plans_respect_built_transport_support(self):
        for name in ("ur5e_arm.yaml", "ur5e_robotiq.yaml", "ur5e_robotiq_remote.yaml"):
            with self.subTest(plan=name):
                result = subprocess.run(
                    [CLI, "validate", str(ROOT / "examples/simulation" / name)],
                    capture_output=True, text=True, timeout=10,
                )
                if HAS_LOCAL or name.endswith("_remote.yaml"):
                    self.assertEqual(result.returncode, 0, result.stderr)
                else:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("local MuJoCo support not built", result.stderr)

    def test_model_roles_accept_shared_prefix_and_all_supported_combinations(self):
        for item in self.mapping["arm"] + self.mapping["effector"]:
            item["joint"] = "robot_" + item["joint"]
            item["actuator"] = "robot_" + item["actuator"]
        for item in self.plan["device"]["arm"]["joints"]:
            item["name"] = "robot_" + item["name"]
        for arm, effector in itertools.product(
            ("mujoco_ur5e", "mujoco_mapped_arm"),
            ("mujoco_robotiq_2f85", "mujoco_mapped_effector"),
        ):
            with self.subTest(arm=arm, effector=effector):
                self.plan["device"]["arm"]["driver"] = arm
                self.plan["device"]["effector"]["driver"] = effector
                result = self.validate()
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_ur5e_rejects_custom_joint_count_but_mapped_role_accepts_it(self):
        self.mapping["arm"] = self.mapping["arm"][:2]
        self.plan["device"]["arm"]["joints"] = self.plan["device"]["arm"]["joints"][:2]
        result = self.validate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("mujoco_ur5e requires six", result.stderr)
        self.plan["device"]["arm"]["driver"] = "mujoco_mapped_arm"
        self.assertEqual(self.validate().returncode, 0)

    def test_ur5e_rejects_reordered_or_inconsistently_prefixed_model(self):
        original_mapping = copy.deepcopy(self.mapping)
        original_plan = copy.deepcopy(self.plan)
        for change in ("order", "actuator", "prefix"):
            with self.subTest(change=change):
                self.mapping = copy.deepcopy(original_mapping)
                self.plan = copy.deepcopy(original_plan)
                if change == "order":
                    self.mapping["arm"][1], self.mapping["arm"][2] = self.mapping["arm"][2], self.mapping["arm"][1]
                    joints = self.plan["device"]["arm"]["joints"]
                    joints[1], joints[2] = joints[2], joints[1]
                elif change == "actuator":
                    self.mapping["arm"][1]["actuator"] = "different_servo"
                else:
                    self.mapping["arm"][1]["joint"] = "other_shoulder_lift_joint"
                    self.plan["device"]["arm"]["joints"][1]["name"] = "other_shoulder_lift_joint"
                result = self.validate()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("mujoco_ur5e requires ordered", result.stderr)

    def test_robotiq_rejects_other_joint_actuator_and_endpoints(self):
        original = copy.deepcopy(self.mapping["effector"][0])
        for key, value in (("joint", "finger_joint"), ("actuator", "another_servo"),
                           ("joint_closed", 1), ("joint_open", -0.2),
                           ("control_closed", 1), ("control_open", -1)):
            with self.subTest(field=key):
                self.mapping["effector"][0] = dict(original, **{key: value})
                self.assertNotEqual(self.validate().returncode, 0)
        self.plan["device"]["effector"]["driver"] = "mujoco_mapped_effector"
        self.assertEqual(self.validate().returncode, 0)

    def test_robotiq_rejects_extra_channels(self):
        second = dict(self.mapping["effector"][0], channel="other", joint="other_joint", actuator="other_servo")
        self.mapping["effector"].append(second)
        self.plan["device"]["effector"]["channels"].append("other")
        result = self.validate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("mujoco_robotiq_2f85 requires one", result.stderr)
        self.plan["device"]["effector"]["driver"] = "mujoco_mapped_effector"
        self.assertEqual(self.validate().returncode, 0)


if __name__ == "__main__":
    unittest.main()
