# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline inverse kinematics and Cartesian line tests on the installed robot descriptions
(numpy)."""

import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    instance = importlib.util.module_from_spec(spec)
    sys.modules[name] = instance
    spec.loader.exec_module(instance)
    return instance


fk = module("spacemit_manipulation.urdf_fk", "capabilities/kinematics/python/urdf_fk.py")
ik = module("spacemit_manipulation.urdf_ik", "capabilities/kinematics/python/urdf_ik.py")


def load(name):
    path = ROOT / "descriptions/robots" / name / "model.json"
    config = json.loads(path.read_text())
    return fk.UrdfKinematics(
        path.parent / config["urdf"], config["base"], config["tip"], config["joints"]
    )


def sample(model, generator, margin=0.2):
    return [generator.uniform(low + margin, high - margin) for low, high in model.limits]


class KinematicsTests(unittest.TestCase):
    models = {name: load(name) for name in ("so101", "nero")}

    def test_jacobian_matches_finite_differences(self):
        generator = random.Random(1)
        for name, model in self.models.items():
            joints = sample(model, generator)
            tip, jacobian = ik.jacobian(model, joints)
            for index in range(len(joints)):
                moved = list(joints)
                moved[index] += 1e-6
                other = ik.matrix(model.forward(moved, check_limits=False))
                linear = (other[:3, 3] - tip[:3, 3]) / 1e-6
                angular = ik.rotation_vector(other[:3, :3] @ tip[:3, :3].T) / 1e-6
                np.testing.assert_allclose(
                    jacobian[:, index],
                    np.concatenate([linear, angular]),
                    atol=1e-5,
                    err_msg=f"{name} joint {index}",
                )

    def test_pose_helpers_round_trip(self):
        generator = random.Random(2)
        for _ in range(50):
            angles = [
                generator.uniform(-3, 3),
                generator.uniform(-1.5, 1.5),
                generator.uniform(-3, 3),
            ]
            rotation = ik.rpy_matrix(*angles)
            np.testing.assert_allclose(ik.rpy_matrix(*ik.rpy(rotation)), rotation, atol=1e-9)
            vector = ik.rotation_vector(rotation)
            np.testing.assert_allclose(ik.rotation_matrix(vector), rotation, atol=1e-9)
        half_turn = ik.rotation_matrix([0, 0, math.pi])  # The skew part vanishes at pi.
        self.assertAlmostEqual(float(np.linalg.norm(ik.rotation_vector(half_turn))), math.pi)

    def test_reachable_positions_converge_within_limits(self):
        generator = random.Random(3)
        for name, model in self.models.items():
            bounds = ik.joint_limits(model)
            for _ in range(20):
                goal = sample(model, generator)
                target = ik.matrix(model.forward(goal))
                seed = [value + generator.uniform(-0.3, 0.3) for value in goal]
                result = ik.solve(model, target, seed)
                self.assertTrue(result["converged"], (name, result))
                self.assertLessEqual(result["position_error_m"], ik.POSITION_TOLERANCE_M)
                reached = ik.matrix(
                    model.forward(result["joints_rad"])
                )  # Raises outside the URDF limits.
                self.assertLessEqual(
                    np.linalg.norm(reached[:3, 3] - target[:3, 3]), ik.POSITION_TOLERANCE_M
                )
                self.assertTrue(
                    np.all(result["joints_rad"] >= bounds[:, 0])
                    and np.all(result["joints_rad"] <= bounds[:, 1])
                )

    def test_full_pose_converges_with_orientation(self):
        generator = random.Random(4)
        for name, model in self.models.items():
            for _ in range(10):
                goal = sample(model, generator)
                target = ik.matrix(model.forward(goal))
                seed = [value + generator.uniform(-0.2, 0.2) for value in goal]
                result = ik.solve(model, target, seed, orientation_weight=1.0)
                self.assertTrue(result["converged"], (name, result))
                self.assertLessEqual(result["orientation_error_rad"], ik.ORIENTATION_TOLERANCE_RAD)

    def test_unreachable_target_and_configured_limits(self):
        model = self.models["so101"]
        far = ik.pose([2.0, 0.0, 0.2])
        result = ik.solve(model, far, [0] * 5)
        self.assertFalse(result["converged"])
        self.assertGreater(result["position_error_m"], 1.0)
        # Configured limits narrow the URDF: a goal outside them is not reached.
        goal = [0.8, 0.3, 0.3, 0.2, 0.0]
        narrow = [[-0.1, 0.1]] + [list(pair) for pair in model.limits[1:]]
        result = ik.solve(model, ik.matrix(model.forward(goal)), [0] * 5, limits=narrow)
        self.assertLessEqual(abs(result["joints_rad"][0]), 0.1)
        self.assertFalse(result["converged"])
        with self.assertRaises(ValueError):
            ik.solve(
                model, far, [0] * 5, limits=[[4, 5]] + [list(pair) for pair in model.limits[1:]]
            )
        with self.assertRaises(ValueError):
            ik.solve(model, far, [0] * 4)
        with self.assertRaises(ValueError):
            ik.solve(model, far, [0] * 5, orientation_weight=2)

    def test_line_is_straight_and_continuous(self):
        for name, model in self.models.items():
            start = (
                [0.1, 0.3, 0.4, 0.2, 0.1]
                if name == "so101"
                else [0.1, 0.5, 0.1, 1.2, 0.1, 0.3, 0.1]
            )
            origin = ik.matrix(model.forward(start))
            target = ik.pose(origin[:3, 3] + [0.03, -0.02, 0.02], orientation=origin[:3, :3])
            weight = 0.0 if name == "so101" else 1.0
            waypoints = ik.line(model, start, target, step_m=0.005, orientation_weight=weight)
            self.assertEqual(len(waypoints), math.ceil(np.linalg.norm([0.03, -0.02, 0.02]) / 0.005))
            direction = (target[:3, 3] - origin[:3, 3]) / np.linalg.norm(
                target[:3, 3] - origin[:3, 3]
            )
            previous = start
            for joints in waypoints:
                point = ik.matrix(model.forward(joints))[:3, 3] - origin[:3, 3]
                self.assertLess(
                    np.linalg.norm(point - direction * (point @ direction)), 0.001, name
                )
                self.assertLessEqual(
                    max(abs(a - b) for a, b in zip(joints, previous)),
                    ik.LINE_MAXIMUM_JOINT_JUMP_RAD,
                )
                previous = joints
            end = ik.matrix(model.forward(waypoints[-1]))
            self.assertLessEqual(
                np.linalg.norm(end[:3, 3] - target[:3, 3]), ik.POSITION_TOLERANCE_M
            )

    def test_line_rejects_unreachable_and_oversized_lines_before_motion(self):
        model = self.models["so101"]
        start = [0.1, 0.3, 0.4, 0.2, 0.1]
        with self.assertRaisesRegex(ValueError, "unreachable"):
            ik.line(model, start, ik.pose([1.0, 0.0, 0.2]), step_m=1.0)
        with self.assertRaisesRegex(ValueError, "IK branch"):  # 5 cm steps jump between solutions.
            ik.line(model, start, ik.pose([1.0, 0.0, 0.2]), step_m=0.05)
        with self.assertRaisesRegex(ValueError, "waypoints"):
            ik.line(model, start, ik.pose([1.0, 0.0, 0.2]), step_m=0.001)
        with self.assertRaises(ValueError):
            ik.line(model, start, ik.pose([0.2, 0.0, 0.2]), step_m=0)


if __name__ == "__main__":
    unittest.main()
