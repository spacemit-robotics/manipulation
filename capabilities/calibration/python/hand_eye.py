# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Device-independent eye-to-hand solve and held-out quality evaluation."""

import copy
import math

import numpy as np

from .calibration_geometry import average, distance, inverse, matrix, matrix_pose


def diversity(transforms):
    """Reject calibration data without rotation about at least two independent axes."""
    import cv2

    vectors = [
        cv2.Rodrigues(transforms[0][:3, :3].T @ value[:3, :3])[0].ravel()
        for value in transforms[1:]
    ]
    singular = np.linalg.svd(np.array(vectors), compute_uv=False)
    if len(singular) < 2 or singular[1] < 0.1:
        raise ValueError("Insufficient rotation diversity: use rotations about at least two axes")
    return singular.tolist()


def fit(base_tool, camera_target):
    """Park solve with eye-to-hand input inversion; return T_base_camera."""
    import cv2

    diversity(base_tool)
    tool_base = [inverse(value) for value in base_tool]
    rotation, translation = cv2.calibrateHandEye(
        [value[:3, :3] for value in tool_base],
        [value[:3, 3] for value in tool_base],
        [value[:3, :3] for value in camera_target],
        [value[:3, 3] for value in camera_target],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    output = np.eye(4)
    output[:3, :3], output[:3, 3] = rotation, np.asarray(translation).ravel()
    return matrix(output)


def solve(samples, tcp, max_position_m=0.005, max_angle_deg=2.0, *, residual=distance):
    """Solve stationary samples without assuming a robot model or joint count.

    Every fourth sample is held out before the final full-data fit. Both held-out
    and full-data residuals must pass. TCP must remain fixed for the whole session.
    The result preserves the established eye-to-hand JSON report format.
    """
    if not 12 <= len(samples) <= 1000:
        raise ValueError("Need 12..1000 samples, including held-out validation poses")
    if not (0 < max_position_m <= 0.015 and 0 < max_angle_deg <= 5.0):
        raise ValueError("Invalid residual thresholds; limits are 15 mm / 5 degrees")
    tcp = matrix(tcp)
    base_tool = [matrix(sample["T_base_flange"]) @ tcp for sample in samples]
    camera_target = [matrix(sample["T_camera_board"]) for sample in samples]
    holdout = [index for index in range(len(samples)) if index % 4 == 3]
    training = [index for index in range(len(samples)) if index not in holdout]
    trained = fit([base_tool[i] for i in training], [camera_target[i] for i in training])
    tool_target = average([inverse(base_tool[i]) @ trained @ camera_target[i] for i in training])
    errors = [residual(trained @ camera_target[i], base_tool[i] @ tool_target) for i in holdout]
    fitted = fit(base_tool, camera_target)
    tool_target = average(
        [inverse(first) @ fitted @ second for first, second in zip(base_tool, camera_target)]
    )
    residuals = [
        residual(fitted @ second, first @ tool_target)
        for first, second in zip(base_tool, camera_target)
    ]
    passed = all(
        math.isfinite(position)
        and math.isfinite(angle)
        and 0 <= position <= max_position_m
        and 0 <= angle <= max_angle_deg
        for position, angle in errors + residuals
    )
    return {
        "schema": 1,
        "mode": "eye_to_hand",
        "quality_passed": bool(passed),
        "direction": "T_base_camera maps camera optical coordinates into arm base",
        "T_base_camera": fitted.tolist(),
        "T_tcp_board": tool_target.tolist(),
        "T_flange_tcp": tcp.tolist(),
        "pose_base_camera_m_rad": matrix_pose(fitted),
        "rotation_diversity_singular_values": diversity(base_tool),
        "holdout_indices": holdout,
        "holdout_errors_m_deg": errors,
        "all_sample_errors_m_deg": residuals,
        "thresholds": {"position_m": max_position_m, "angle_deg": max_angle_deg},
    }


def maximum_residual(report):
    """Return the worst (position_m, angle_deg) over both residual groups, or None.

    Used only to make a quality failure legible: it never decides the verdict, so a
    malformed group yields None and the caller falls back to the plain message.
    """
    worst_position = worst_angle = None
    for key in ("holdout_errors_m_deg", "all_sample_errors_m_deg"):
        group = report.get(key)
        if not isinstance(group, list):
            continue
        for item in group:
            if (
                not isinstance(item, (list, tuple))
                or len(item) != 2
                or not all(
                    type(value) in (float, int) and math.isfinite(value)
                    for value in item
                )
            ):
                continue
            worst_position = item[0] if worst_position is None else max(worst_position, item[0])
            worst_angle = item[1] if worst_angle is None else max(worst_angle, item[1])
    if worst_position is None:
        return None
    return worst_position, worst_angle


def validate_report(report):
    """Recheck transforms and every recorded residual before publishing an extrinsic."""
    if report.get("schema") != 1 or report.get("mode") != "eye_to_hand":
        raise ValueError("Only a quality-passed eye-to-hand report can be published")
    if report.get("quality_passed") is not True:
        limits = report.get("thresholds") or {}
        worst = maximum_residual(report)
        raise ValueError(
            "Report quality failed: worst residual "
            f"{worst[0] * 1000:.2f} mm / {worst[1]:.2f} deg against "
            f"{limits.get('position_m', 0) * 1000:.1f} mm / "
            f"{limits.get('angle_deg', 0):.1f} deg; the publication ceiling is "
            "15 mm / 5 deg, so either re-solve with wider thresholds or capture "
            "more poses"
            if worst else
            "Only a quality-passed eye-to-hand report can be published")

    matrix(report["T_base_camera"])
    matrix(report["T_flange_tcp"])
    limits = report["thresholds"]
    position, angle = limits["position_m"], limits["angle_deg"]
    if not (0 < position <= 0.015 and 0 < angle <= 5):
        raise ValueError("Invalid report quality thresholds")
    for key in ("holdout_errors_m_deg", "all_sample_errors_m_deg"):
        values = report.get(key)
        minimum = 3 if key == "holdout_errors_m_deg" else 12
        if (
            not isinstance(values, list)
            or len(values) < minimum
            or any(
                len(item) != 2
                or not all(math.isfinite(value) for value in item)
                or not 0 <= item[0] <= position
                or not 0 <= item[1] <= angle
                for item in values
            )
        ):
            raise ValueError("Report residual fails quality gate")


def evaluate_residuals(report, max_position_m=0.005, max_angle_deg=2.0):
    """Re-evaluate saved evidence without changing transforms or historical inputs.

    This is an assessment, not publication: validate_report still enforces the
    publication ceiling and transform contract. Missing or invalid residuals fail.
    """
    if not isinstance(report, dict):
        raise ValueError("A calibration report is required")
    output = copy.deepcopy(report)
    groups = [output.get(key) for key in ("all_sample_errors_m_deg", "holdout_errors_m_deg")]
    complete = all(isinstance(group, list) and group for group in groups)
    if complete:
        complete = all(
            isinstance(pair, (list, tuple))
            and len(pair) == 2
            and all(
                type(value) in (float, int) and math.isfinite(value) and value >= 0
                for value in pair
            )
            for group in groups
            for pair in group
        )
    output["quality_passed"] = bool(
        complete
        and all(
            position <= max_position_m and angle <= max_angle_deg
            for group in groups
            for position, angle in group
        )
    )
    output["thresholds"] = {"position_m": max_position_m, "angle_deg": max_angle_deg}
    return output
