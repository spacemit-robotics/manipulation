#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Shared calibration contracts with synthetic sensors; no camera or robot access."""

from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))
import spacemit_manipulation  # noqa: E402

for directory in (
    "capabilities/calibration/python",
    "capabilities/calibration/cli",
    "capabilities/kinematics/python",
):
    spacemit_manipulation.__path__.append(str(ROOT / directory))

from spacemit_manipulation.calibration_geometry import inverse, pose_matrix  # noqa: E402
from spacemit_manipulation.calibration_session import (  # noqa: E402
    CalibrationSession,
    device_identity,
    export_perceptive,
    load_config,
    read_dataset,
    session_identity,
    solve_session,
)
from spacemit_manipulation.sampling import FeedbackPort, capture_sample  # noqa: E402
from spacemit_manipulation.workbench import export_legacy, promote, save_new  # noqa: E402
from spacemit_manipulation import collect  # noqa: E402


def synthetic_samples():
    """Two rotated frames with exact closure and a sufficiently excited robot motion set."""
    random = np.random.default_rng(123)
    base_camera = pose_matrix([0.42, -0.18, 0.7, 0.2, -0.3, 0.4])
    flange_board = pose_matrix([0.02, 0.03, 0.11, -0.4, 0.2, 0.1])
    samples = []
    for _ in range(24):
        flange = pose_matrix([*random.uniform(-0.3, 0.3, 3), *random.uniform(-0.8, 0.8, 3)])
        samples.append(
            {
                "T_base_flange": flange.tolist(),
                "T_camera_board": (inverse(base_camera) @ flange @ flange_board).tolist(),
            }
        )
    return base_camera, samples


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="calibration-session-")
        self.root = Path(self.temporary.name)
        self.addCleanup(self.temporary.cleanup)
        (self.root / "device.yaml").write_text("schema_version: 1\n", encoding="utf-8")
        (self.root / "arm.urdf").write_text("<robot name='test'/>", encoding="utf-8")
        save_new(
            self.root / "model.json",
            {"urdf": "arm.urdf", "base": "base", "tip": "flange", "joints": ["j1", "j2"]},
        )
        self.provenance = {
            "camera": {
                "serial": "camera-1",
                "frame": "color_optical",
                "camera_matrix": np.eye(3).tolist(),
            },
            "board": {"type": "charuco", "square_length_m": 0.02},
        }
        self.device = device_identity(
            self.root / "device.yaml", self.root / "model.json", installation_id="table-1"
        )
        self.identity = session_identity(self.device, self.provenance)

    def collect(self):
        session = CalibrationSession(self.root / "session", self.identity, self.provenance)
        expected, samples = synthetic_samples()
        for sample in samples:
            session.append(sample)
        session.finish()
        return session, expected

    def test_collect_solve_promote_and_application_exports(self):
        session, expected = self.collect()
        report = solve_session(session.path)
        self.assertTrue(report["quality_passed"])
        np.testing.assert_allclose(report["T_base_camera"], expected, atol=1e-7)
        save_new(session.path / "candidate.json", report)
        promote(session.path / "candidate.json", session.path / "active.json")
        export_legacy(
            session.path / "active.json",
            session.path / "provenance.json",
            session.path / "extrinsic.json",
        )
        export_perceptive(report, session.path / "perceptive.json", "realsense", "table-1")
        exported = json.loads((session.path / "perceptive.json").read_text())
        pose = exported["calibration"]["realsense"]["T_base_camera"]
        np.testing.assert_allclose(
            pose_matrix(pose["translation"] + pose["rotation"]), expected, atol=1e-7
        )
        self.assertEqual(report["session_identity"], self.identity)
        with self.assertRaises(FileExistsError):
            export_perceptive(report, session.path / "perceptive.json", "realsense", "table-1")

    def test_incomplete_and_mixed_sessions_are_rejected(self):
        session = CalibrationSession(self.root / "session", self.identity, self.provenance)
        with self.assertRaisesRegex(ValueError, "incomplete"):
            read_dataset(session.path)
        _, samples = synthetic_samples()
        with self.assertRaisesRegex(ValueError, "different"):
            session.append(dict(samples[0], session_identity_sha256="other-installation"))
        for sample in samples:
            session.append(sample)
        session.finish()
        record = json.loads((session.path / "sample_0000.json").read_text())
        record["session_identity_sha256"] = "other-installation"
        (session.path / "sample_0000.json").write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError, "different"):
            solve_session(session.path)

    def test_export_requires_matching_installation_and_passed_residuals(self):
        session, _ = self.collect()
        report = solve_session(session.path)
        with self.assertRaisesRegex(ValueError, "installation"):
            export_perceptive(report, self.root / "wrong.json", "realsense", "table-2")
        report["holdout_errors_m_deg"][0] = [0.05, 0]
        with self.assertRaisesRegex(ValueError, "residual"):
            export_perceptive(report, self.root / "wrong.json", "realsense", "table-1")
        self.assertFalse((self.root / "wrong.json").exists())

    def test_camera_board_and_model_are_bound_to_session(self):
        session, _ = self.collect()
        manifest = json.loads((session.path / "manifest.json").read_text())
        manifest["camera"] = dict(manifest["camera"], serial="other-camera")
        (session.path / "manifest.json").write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "identity"):
            read_dataset(session.path)
        with self.assertRaisesRegex(ValueError, "frame"):
            session_identity(
                self.device,
                self.provenance,
                {
                    "base": "other-base",
                    "flange": "flange",
                    "camera": "color_optical",
                    "target": "board",
                },
            )

    def test_config_resolves_relative_resources_and_has_no_robot_model_branch(self):
        config = {
            "schema_version": 1,
            "installation_id": "table-1",
            "device_plan": "device.yaml",
            "model": "model.json",
            "sampler": {"options": {"board": "board.json", "serial": "camera-1"}},
        }
        save_new(self.root / "config.json", config)
        loaded = load_config(self.root / "config.json")
        self.assertEqual(loaded["device_plan"], self.root / "device.yaml")
        self.assertEqual(loaded["model"], self.root / "model.json")
        self.assertEqual(loaded["sampler"]["options"]["board"], str(self.root / "board.json"))
        self.assertEqual(loaded["frames"]["flange"], "flange")

    def test_old_dice_session_remains_readable_but_not_exported_as_bound(self):
        directory = self.root / "legacy"
        directory.mkdir()
        save_new(
            directory / "manifest.json",
            {
                "schema": 1,
                "mode": "eye_to_hand",
                "T_flange_tcp": np.eye(4).tolist(),
                "tcp": "flange",
                **self.provenance,
            },
        )
        _, samples = synthetic_samples()
        for index, sample in enumerate(samples):
            save_new(directory / f"sample_{index:04d}.json", sample)
        report = solve_session(directory)
        self.assertTrue(report["quality_passed"])
        with self.assertRaisesRegex(ValueError, "installation"):
            export_perceptive(report, self.root / "wrong.json", "realsense", "table-1")

    def test_read_only_feedback_accepts_five_and_seven_joint_models(self):
        for count in (5, 7):
            client, model = Mock(), Mock()
            client.state.return_value = {
                "arm_sample_id": 1,
                "moving": False,
                "fault": 0,
                "joints_rad": [0.1] * count,
                "source_time_s": 1,
            }
            port = FeedbackPort(client, model)
            self.assertEqual(len(port.read_joints()["joints_rad"]), count)
            self.assertEqual([call[0] for call in client.method_calls], ["state"])
            client.state.return_value["moving"] = True
            with self.assertRaisesRegex(RuntimeError, "stationary"):
                port.read_joints()

    def test_shared_capture_keeps_callbacks_and_best_frame(self):
        arm, camera, detector = Mock(), Mock(), Mock()
        arm.read.return_value = np.eye(4)
        arm.read_joints.return_value = {"joints_rad": [0.0] * 5, "joints_deg": [0.0] * 5}
        camera.capture.return_value = np.zeros((10, 10, 3), np.uint8)
        detector.detect.side_effect = [
            (np.eye(4), {"reprojection_rms_px": error}, camera.capture.return_value)
            for error in (0.3, 0.1, 0.2)
        ]
        check, observe = Mock(), Mock()
        with patch("spacemit_manipulation.sampling.time.sleep"):
            sample, _, _ = capture_sample(
                arm, camera, detector, frame_check=check, frame_observer=observe
            )
        self.assertEqual(sample["selected_frame_index"], 1)
        self.assertEqual(check.call_count, 8)
        self.assertEqual(observe.call_count, 3)
        self.assertEqual(len(sample["joints_rad"]), 5)

    def test_configured_collector_releases_read_only_device_between_samples(self):
        for joint_count in (5, 7):
            config_path = self.root / f"collection_{joint_count}.json"
            model_path = self.root / f"model_{joint_count}.json"
            save_new(
                model_path,
                {
                    "urdf": "arm.urdf",
                    "base": "base",
                    "tip": "flange",
                    "joints": [f"j{index}" for index in range(joint_count)],
                },
            )
            output = self.root / f"captured_{joint_count}"
            save_new(
                config_path,
                {
                    "schema_version": 1,
                    "installation_id": "table-1",
                    "device_plan": "device.yaml",
                    "model": model_path.name,
                    "sampler": {"module": "test_sampler", "options": {}},
                },
            )
            sampler = Mock()
            sampler.provenance.return_value = self.provenance
            sampler.dependencies.return_value = []
            _, samples = synthetic_samples()
            observed_counts = []

            def capture(port, prefix):
                joints = port.read_joints()["joints_rad"]
                observed_counts.append(len(joints))
                return dict(samples[len(observed_counts) - 1], joints_rad=joints)

            sampler.capture.side_effect = capture
            client = Mock()
            client.state.return_value = {
                "arm_sample_id": 1,
                "joints_rad": [0.0] * joint_count,
                "source_time_s": 1,
                "moving": False,
                "fault": 0,
            }
            factory = Mock()
            factory.return_value.__enter__ = Mock(return_value=client)
            factory.return_value.__exit__ = Mock()
            arguments = [
                "collect",
                "--config",
                str(config_path),
                "--device-executable",
                "mock-device",
                "--output",
                str(output),
                "--count",
                "12",
            ]
            with ExitStack() as patches:
                patches.enter_context(patch.object(sys, "argv", arguments))
                patches.enter_context(patch("builtins.input", return_value="CAPTURE"))
                patches.enter_context(patch.object(collect, "UrdfKinematics"))
                patches.enter_context(
                    patch.object(
                        collect.importlib,
                        "import_module",
                        return_value=Mock(create_sampler=Mock(return_value=sampler)),
                    )
                )
                patches.enter_context(patch.object(collect, "DeviceClient", factory))
                collect.main()
            self.assertEqual(observed_counts, [joint_count] * 12)
            self.assertEqual(factory.call_count, 12)
            self.assertTrue(
                all(call.kwargs == {"allow_motion": False} for call in factory.call_args_list)
            )
            self.assertEqual(factory.return_value.__exit__.call_count, 12)
            self.assertEqual(len(read_dataset(output)[1]), 12)
            sampler.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
