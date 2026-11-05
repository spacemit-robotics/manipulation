#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""ChArUco detection and stationary sampling on rendered images. No camera or arm."""

import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
try:
    import cv2
    import numpy as np
except Exception as error:  # noqa: BLE001
    # An unusable OpenCV/NumPy pair means there is nothing to test here.
    print(f"skipped: OpenCV/NumPy unavailable ({error})")
    sys.exit(0)

BOARD = dict(
    type="charuco",
    dictionary="4x4_50",
    squares_x=4,
    squares_y=5,
    square_length_m=0.0215,
    marker_length_m=0.015,
    legacy_pattern=False,
    min_corners=6,
    max_reprojection_px=1.0,
)
camera_matrix = np.array([[900.0, 0, 480], [0, 900.0, 360], [0, 0, 1]])
dist_coeffs = np.zeros(5)
PPM = 4000  # Board image pixels per metre.


def pose(rx, ry, rz, x, y, z):
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(np.array([rx, ry, rz], float))[0]
    transform[:3, 3] = [x, y, z]
    return transform


def render(detector, camera_board):
    """Camera image of the board at camera_board, by warping the flat board image."""
    size = (round(4 * 0.0215 * PPM), round(5 * 0.0215 * PPM))
    board = detector.board
    draw = board.generateImage if hasattr(board, "generateImage") else board.draw
    flat = draw(size, marginSize=0)
    to_metric = np.diag([1 / PPM, 1 / PPM, 1.0])
    homography = camera_matrix @ camera_board[:3, [0, 1, 3]] @ to_metric
    image = cv2.warpPerspective(
        flat, homography, (960, 720), flags=cv2.INTER_LINEAR, borderValue=255
    )
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def difference(first, second):
    relative = first[:3, :3].T @ second[:3, :3]
    angle = math.degrees(math.acos(max(-1, min(1, (np.trace(relative) - 1) / 2))))
    return float(np.linalg.norm(first[:3, 3] - second[:3, 3])), angle


class Camera:
    def __init__(self, image):
        self.image, self.camera_matrix, self.dist_coeffs = image, camera_matrix, dist_coeffs
        self.info = dict(
            backend="realsense",
            frame="color_optical",
            serial="rendered",
            width=960,
            height=720,
            camera_matrix=camera_matrix.tolist(),
            dist_coeffs=dist_coeffs.tolist(),
        )

    def capture(self):
        return self.image.copy()

    def close(self):
        pass


class Port:
    def __init__(self, moving=False):
        self.moving, self.calls, self.sample = moving, 0, 0

    def read(self):
        self.calls += 1
        flange = np.eye(4)
        flange[0, 3] = 0.001 * self.calls if self.moving else 0
        return flange

    def read_joints(self):
        self.sample += 1
        return dict(
            joints_rad=[0.0] * 7,
            joints_deg=[0.0] * 7,
            sdk_timestamp=self.sample,
            observed_epoch_s=0,
        )


class CharucoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = Path(tempfile.mkdtemp(prefix="charuco-"))
        package = cls.temporary / "calib"
        package.mkdir()
        (package / "__init__.py").write_text("")
        for name in ("charuco.py", "sampling.py", "charuco_sampler.py"):
            shutil.copy(ROOT / "capabilities/calibration/python" / name, package / name)
        sys.path.insert(0, str(cls.temporary))
        global charuco, sampling, charuco_sampler
        from calib import charuco, charuco_sampler, sampling  # noqa: E402

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.temporary)

    def test_relative_pose_is_recovered(self):
        detector = charuco.CharucoDetector(BOARD)
        first, second = (
            pose(0.25, -0.2, 0.1, -0.05, -0.04, 0.42),
            pose(-0.2, 0.3, -0.15, 0.02, -0.03, 0.38),
        )
        found = []
        for truth in (first, second):
            transform, quality, _ = detector.detect(
                render(detector, truth), camera_matrix, dist_coeffs
            )
            self.assertGreaterEqual(quality["corners"], 6)
            self.assertLess(quality["reprojection_rms_px"], 0.5)
            found.append(transform)
        position, angle = difference(
            found[0] @ np.linalg.inv(found[1]), first @ np.linalg.inv(second)
        )
        self.assertLess(position, 0.002)
        self.assertLess(angle, 0.5)

    def test_measured_size_scales_the_board(self):
        image = render(charuco.CharucoDetector(BOARD), pose(0.2, -0.1, 0.0, -0.04, -0.05, 0.40))
        nominal = charuco.CharucoDetector(BOARD).detect(image, camera_matrix, dist_coeffs)[0]
        measured = charuco.CharucoDetector(BOARD, (0.086 * 1.02, 0.1075 * 1.02)).detect(
            image, camera_matrix, dist_coeffs
        )[0]
        self.assertAlmostEqual(measured[2, 3] / nominal[2, 3], 1.02, places=2)

    def test_no_board_and_roi_are_reported(self):
        detector = charuco.CharucoDetector(BOARD)
        with self.assertRaisesRegex(ValueError, "no ChArUco markers"):
            detector.detect(np.full((720, 960, 3), 255, np.uint8), camera_matrix, dist_coeffs)
        hidden = charuco.CharucoDetector(dict(BOARD, image_roi_xyxy=[0, 0, 100, 100]))
        with self.assertRaises(ValueError):
            hidden.detect(
                render(hidden, pose(0.2, -0.1, 0.0, -0.04, -0.05, 0.40)), camera_matrix, dist_coeffs
            )

    def test_stationary_capture_and_motion_rejection(self):
        detector = charuco.CharucoDetector(BOARD)
        camera = Camera(render(detector, pose(0.2, -0.1, 0.0, -0.04, -0.05, 0.40)))
        record, _, _ = sampling.capture_sample(Port(), camera, detector, discard=2)
        self.assertEqual(len(record["frame_joint_observations"]), 3)
        self.assertNotIn("corner_pixels", record["quality"])
        with self.assertRaisesRegex(ValueError, "moved"):
            sampling.capture_sample(Port(moving=True), camera, detector, discard=2)
        with self.assertRaisesRegex(ValueError, "window"):
            sampling.capture_sample(Port(), camera, detector, window=[0, 0, 200, 200], discard=2)

    def test_sampler_writes_new_evidence_only(self):
        board_file = self.temporary / "board.json"
        board_file.write_text(__import__("json").dumps(BOARD))
        camera = Camera(
            render(charuco.CharucoDetector(BOARD), pose(0.2, -0.1, 0.0, -0.04, -0.05, 0.40))
        )
        sampler = charuco_sampler.CharucoSampler(dict(board=str(board_file)), camera=camera)
        prefix = self.temporary / "sample_0000"
        record = sampler.capture(Port(), prefix)
        self.assertTrue(
            Path(f"{prefix}.png").is_file() and Path(f"{prefix}_detected.png").is_file()
        )
        self.assertEqual(len(record["T_camera_board"]), 4)
        with self.assertRaises(FileExistsError):
            sampler.capture(Port(), prefix)
        self.assertEqual(sampler.provenance()["camera"]["serial"], "rendered")
        with self.assertRaises(ValueError):
            charuco_sampler.CharucoSampler(dict(board=str(board_file), typo="1"), camera=camera)


if __name__ == "__main__":
    unittest.main()
