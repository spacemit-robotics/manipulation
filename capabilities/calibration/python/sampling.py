# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Stationary eye-to-hand sampling: fresh arm feedback bracketing several camera frames.

Device access goes only through DeviceClient.state(); nothing here moves the arm.
"""

import math
import time


class FeedbackPort:
    """Distinct fresh stationary arm feedback and its URDF flange pose."""

    def __init__(self, client, model):
        self.client, self.model, self.last = client, model, 0

    def read_joints(self):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            state = self.client.state(timeout=0.5)
            if state["moving"] or state["fault"]:
                raise RuntimeError("arm must remain stationary and healthy during calibration")
            if state["arm_sample_id"] > self.last:
                self.last = state["arm_sample_id"]
                return dict(
                    joints_rad=state["joints_rad"],
                    joints_deg=[math.degrees(v) for v in state["joints_rad"]],
                    sdk_timestamp=state["source_time_s"],
                    observed_epoch_s=time.time(),
                )
            time.sleep(0.01)
        raise RuntimeError("no distinct fresh arm feedback for calibration")

    def read(self):
        import numpy as np

        return np.asarray(
            self.model.forward(self.read_joints()["joints_rad"], check_limits=False)
        ).reshape(4, 4)


def distance(first, second):
    """Translation (m) and rotation (deg) between two 4x4 rigid transforms."""
    import numpy as np

    a, b = np.asarray(first, float), np.asarray(second, float)
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1) / 2))
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), math.degrees(math.acos(cosine))


def assert_still(poses, position_m=0.0005, angle_deg=0.2):
    if len(poses) < 2:
        raise ValueError("need multiple stationary observations")
    if any(p > position_m or a > angle_deg for p, a in (distance(poses[0], t) for t in poses[1:])):
        raise ValueError("arm moved during capture; sample rejected")


def capture_sample(
    port,
    camera,
    detector,
    window=None,
    discard=5,
    frames=3,
    *,
    frame_check=None,
    frame_observer=None,
    keep_corner_pixels=False,
    recording_source="spacemit_cpp_distinct_feedback_urdf_fk",
):
    """Return (record, image, annotated) for one stationary pose, or raise with the reason.

    The flange pose is read before and after every frame; the arm must stay within 0.5 mm / 0.2 deg,
    frames queued before the pose settled are discarded, and the three board poses must agree
    within 2 mm / 1 deg. The frame with the lowest reprojection error is kept.
    """
    import numpy as np

    if type(discard) is not int or discard < 0 or type(frames) is not int or frames < 2:
        raise ValueError("sampling requires nonnegative discard and at least two frames")
    poses = []
    for _ in range(8):
        poses.append(port.read())
        time.sleep(0.05)
    assert_still(poses)
    joints_before = port.read_joints()
    discarded = []
    for _ in range(discard):
        frame = camera.capture()
        discarded.append(port.read_joints())
        if frame_check is not None:
            frame_check(frame)
    started = time.monotonic()
    observations = []
    for frame_index in range(frames):
        poses.append(port.read())
        image = camera.capture()
        captured_at = time.time()
        frame_joints = port.read_joints()
        board = quality = annotated = None
        frame_record = {
            "frame_index": frame_index,
            "time_unix_s": captured_at,
            "T_base_flange_before": poses[-1].tolist(),
            "joint_observation": frame_joints,
        }
        if frame_observer is not None:
            frame_record.update(
                camera=camera.info,
                board=detector.cfg,
                transform_units="metres",
                joint_units="radians_and_degrees",
            )
        try:
            board, quality, annotated = detector.detect(
                image, camera.camera_matrix, camera.dist_coeffs
            )
            if window is not None:
                inside_window(quality["corner_pixels"], window)
            if frame_check is not None:
                frame_check(image, board)
            poses.append(port.read())
            frame_record["T_base_flange_after"] = poses[-1].tolist()
        except BaseException as error:
            if frame_observer is not None:
                frame_observer(
                    image,
                    annotated,
                    dict(
                        frame_record,
                        status="rejected",
                        error=str(error) or type(error).__name__,
                        quality=quality,
                        T_camera_board=None if board is None else board.tolist(),
                    ),
                )
            raise
        if frame_observer is not None:
            frame_observer(
                image,
                annotated,
                dict(
                    frame_record,
                    status="detected",
                    quality=quality,
                    T_camera_board=board.tolist(),
                ),
            )
        observations.append((board, quality, image, annotated, poses[-1], frame_joints))
    assert_still(poses)
    if any(
        p > 0.002 or a > 1.0
        for p, a in (distance(observations[0][0], o[0]) for o in observations[1:])
    ):
        raise ValueError("board observations unstable; sample rejected")
    board, quality, image, annotated, flange, _ = min(
        observations, key=lambda o: o[1]["reprojection_rms_px"]
    )
    joints_after = port.read_joints()
    if np.max(
        np.abs(np.asarray(joints_after["joints_rad"]) - joints_before["joints_rad"])
    ) > math.radians(0.2):
        raise ValueError("joints moved during calibration sample")
    if not keep_corner_pixels:
        quality = {key: value for key, value in quality.items() if key != "corner_pixels"}
    archive_selection = (
        {
            "selected_frame_index": min(
                range(len(observations)),
                key=lambda index: observations[index][1]["reprojection_rms_px"],
            )
        }
        if frame_observer is not None
        else {}
    )
    return (
        dict(
            **archive_selection,
            T_base_flange=flange.tolist(),
            T_camera_board=board.tolist(),
            joints_rad=joints_after["joints_rad"],
            joints_deg=joints_after["joints_deg"],
            joint_observations=dict(before=joints_before, after=joints_after),
            frame_joint_observations=[o[5] for o in observations],
            discarded_frame_joint_observations=discarded,
            joint_recording_source=recording_source,
            quality=quality,
            time_unix_s=time.time(),
            capture_duration_s=time.monotonic() - started,
        ),
        image,
        annotated,
    )


def inside_window(points, window, margin_px=8):
    x0, y0, x1, y1 = window
    if any(
        not (x0 + margin_px <= x <= x1 - margin_px and y0 + margin_px <= y <= y1 - margin_px)
        for x, y in points
    ):
        raise ValueError("calibration board outside the allowed image window")
