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
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

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
module(
    "spacemit_manipulation.operator_support",
    "bindings/python/spacemit_manipulation/operator_support.py",
)
operator_cli = module(
    "spacemit_manipulation.operator_cli", "capabilities/diagnostics/cli/operator_cli.py"
)
client_module = module(
    "spacemit_manipulation.client", "bindings/python/spacemit_manipulation/client.py"
)


def load(name):
    path = ROOT / "descriptions/robots" / name / "model.json"
    config = json.loads(path.read_text())
    return fk.UrdfKinematics(
        path.parent / config["urdf"], config["base"], config["tip"], config["joints"]
    )


def sample(model, generator, margin=0.2):
    return [generator.uniform(low + margin, high - margin) for low, high in model.limits]


class LineMoveDeadlineTests(unittest.TestCase):
    def test_deadline_covers_the_path_profile_and_the_arm_timeout(self):
        # 1.2 rad of largest-joint change at 0.2 rad/s: 1.875 * 1.2 / 0.2 = 11.25 s of profile.
        waypoints = [[0.01 * i, 0.0] for i in range(1, 121)]
        timeout = operator_cli.path_timeout([0.0, 0.0], waypoints, 0.2, 0.5, 10)
        self.assertAlmostEqual(timeout, 11.25 + 10 + 5)
        self.assertEqual(operator_cli.path_timeout([0.0], [[100.0]], 0.2, 0.5, 10), 180)


class PointMoveDeadlineTests(unittest.TestCase):
    def setUp(self):
        self.description = {
            "joints": ["shoulder_pan", "shoulder_lift"],
            "joint_limits_rad": [[-2, 2], [-2, 2]],
            "arm_timeout_s": 10,
            "arm_tolerance_rad": 0.01,
        }
        self.context = SimpleNamespace(
            plan=Mock(return_value=Path("device.yaml")),
            describe=Mock(return_value=self.description),
            binary=Mock(return_value=Path("manipulation-device")),
        )
        client_patch = patch.object(client_module, "DeviceClient")
        self.factory = client_patch.start()
        self.addCleanup(client_patch.stop)
        self.client = self.factory.return_value.__enter__.return_value
        self.client.state.return_value = {"joints_rad": [0.0, 0.0]}
        self.client.capabilities.return_value = {"arm": {"joint_path": False}}
        print_patch = patch("builtins.print")
        print_patch.start()
        self.addCleanup(print_patch.stop)

    def args(self, operation):
        return SimpleNamespace(
            operation=operation, plan=None, joint="shoulder_pan", delta=0.05,
            positions=[0.1, 0.2], speed=0.2, x=0.1, y=0.0, z=0.2,
            rpy=None, orientation_weight=None, seed_pose=None, step_mm=5,
            path_velocity=0.2, path_acceleration=0.5,
        )

    def test_joint_commands_reserve_time_after_the_device_deadline(self):
        for configured, expected in ((0.2, 10), (10, 15), (30, 35), (175, 180)):
            with self.subTest(arm_timeout_s=configured):
                self.description["arm_timeout_s"] = configured
                operator_cli.joint_move(self.context, self.args("joint-step"))
                self.client.jog_arm.assert_called_with(0, 0.05, 0.2, timeout=expected)
                operator_cli.joint_move(self.context, self.args("joint-move"))
                self.client.move_arm.assert_called_with([0.1, 0.2], 0.2, timeout=expected)

    def test_joint_budget_over_client_limit_is_rejected_before_connecting(self):
        self.description["arm_timeout_s"] = 176
        for operation in ("joint-step", "joint-move"):
            with self.subTest(operation=operation):
                with self.assertRaisesRegex(ValueError, "180 s"):
                    operator_cli.joint_move(self.context, self.args(operation))
        self.factory.assert_not_called()

    def test_joint_move_refuses_unreachable_duration_without_dispatching(self):
        self.client.state.return_value = {
            "joints_rad": [0.0, 0.0],
            "controller_limits": [{"max_velocity_rad_s": 1.0}] * 2,
        }
        args = self.args("joint-move")
        args.positions, args.speed = [1.5, 0.0], 0.05
        with self.assertRaisesRegex(ValueError, "more than the arm timeout_s 10"):
            operator_cli.joint_move(self.context, args)
        self.client.move_arm.assert_not_called()
        self.client.jog_arm.assert_not_called()

    def test_joint_move_dispatches_reachable_duration_with_normal_deadline(self):
        self.client.state.return_value = {
            "joints_rad": [0.0, 0.0],
            "controller_limits": [{"max_velocity_rad_s": 1.0}] * 2,
        }
        operator_cli.joint_move(self.context, self.args("joint-move"))
        self.client.move_arm.assert_called_once_with([0.1, 0.2], 0.2, timeout=15)

    def test_joint_move_duration_allows_reaching_the_configured_arrival_tolerance(self):
        self.description["arm_timeout_s"] = 1
        self.client.state.return_value = {
            "joints_rad": [0.0, 0.0],
            "controller_limits": [{"max_velocity_rad_s": 1.0}] * 2,
        }
        # The first target is already within tolerance. The second needs only 0.625 s to enter
        # tolerance, although reaching either exact target would take longer than the deadline.
        for position in (0.009, 0.011):
            with self.subTest(position=position):
                self.client.move_arm.reset_mock()
                args = self.args("joint-move")
                args.positions, args.speed = [position, 0.0], 0.0016
                operator_cli.joint_move(self.context, args)
                self.client.move_arm.assert_called_once_with(
                    [position, 0.0], 0.0016, timeout=10
                )

    def test_joint_move_without_controller_limits_keeps_existing_dispatch(self):
        for limits in (None, [], [{"max_velocity_rad_s": 1.0}]):
            with self.subTest(controller_limits=limits):
                state = {"joints_rad": [0.0, 0.0]}
                if limits is not None:
                    state["controller_limits"] = limits
                self.client.state.return_value = state
                self.client.move_arm.reset_mock()
                args = self.args("joint-move")
                args.positions, args.speed = [1.5, 0.0], 0.05
                operator_cli.joint_move(self.context, args)
                self.client.move_arm.assert_called_once_with([1.5, 0.0], 0.05, timeout=15)

    def test_joint_move_rejects_invalid_speed_without_dispatching(self):
        for limits in (None, [{"max_velocity_rad_s": 1.0}] * 2):
            for speed in (0.0, -0.1, 1.01, math.nan, math.inf, -math.inf):
                with self.subTest(controller_limits=limits, speed=speed):
                    state = {"joints_rad": [0.0, 0.0]}
                    if limits is not None:
                        state["controller_limits"] = limits
                    self.client.state.return_value = state
                    args = self.args("joint-move")
                    args.speed = speed
                    with self.assertRaises(ValueError):
                        operator_cli.joint_move(self.context, args)
        self.client.move_arm.assert_not_called()
        self.client.jog_arm.assert_not_called()

    def test_pose_move_uses_the_same_device_deadline_budget(self):
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "solve", return_value={"converged": True, "joints_rad": [0.1, 0.2]}
        ):
            operator_cli.kinematics(self.context, self.args("pose-move"))
        self.client.move_arm.assert_called_once_with([0.1, 0.2], 0.2, timeout=15)

    def test_line_point_fallback_keeps_handoff_and_final_arrival(self):
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "line", return_value=[[0.01, 0.0], [0.02, 0.0]]
        ):
            operator_cli.kinematics(self.context, self.args("line-move"))
        self.assertEqual(self.client.move_arm.call_args_list, [
            call([0.01, 0.0], 0.2, wait_for_arrival=False, timeout=15),
            call([0.02, 0.0], 0.2, timeout=15),
        ])

    def test_pose_budget_over_client_limit_is_rejected_before_connecting(self):
        self.description["arm_timeout_s"] = 176
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "solve", return_value={"converged": True, "joints_rad": [0.1, 0.2]}
        ):
            with self.assertRaisesRegex(ValueError, "180 s"):
                operator_cli.kinematics(self.context, self.args("pose-move"))
        self.factory.assert_not_called()

    def test_line_point_budget_is_checked_before_the_first_waypoint(self):
        self.description["arm_timeout_s"] = 176
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "line", return_value=[[0.01, 0.0], [0.02, 0.0]]
        ):
            with self.assertRaisesRegex(ValueError, "180 s"):
                operator_cli.kinematics(self.context, self.args("line-move"))
        self.client.move_arm.assert_not_called()

    def test_ik_remains_read_only_with_a_long_device_timeout(self):
        self.description["arm_timeout_s"] = 300
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "solve", return_value={"converged": True, "joints_rad": [0.1, 0.2]}
        ):
            operator_cli.kinematics(self.context, self.args("ik"))
        self.factory.assert_called_once_with(
            Path("manipulation-device"), Path("device.yaml"), allow_motion=False
        )
        self.client.move_arm.assert_not_called()

    def test_continuous_line_keeps_its_existing_profile_budget(self):
        self.description["arm_timeout_s"] = 176
        self.client.capabilities.return_value = {"arm": {"joint_path": True}}
        waypoints = [[0.01, 0.0], [0.02, 0.0]]
        with patch.object(operator_cli, "load_model", return_value=object()), patch.object(
            ik, "line", return_value=waypoints
        ):
            operator_cli.kinematics(self.context, self.args("line-move"))
        self.client.move_arm_path.assert_called_once_with(
            waypoints, 0.2, 0.2, 0.5, timeout=180
        )
        self.client.move_arm.assert_not_called()


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
