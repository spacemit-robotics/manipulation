# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Rigid transforms for hand-eye calibration, in metres and radians."""

import math

import numpy as np


def matrix(value):
    """Validate and return a rigid 4x4 transform; no device conventions are inferred."""
    transform = np.asarray(value, dtype=float)
    if (
        transform.shape != (4, 4)
        or not np.isfinite(transform).all()
        or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
        or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
        or abs(np.linalg.det(transform[:3, :3]) - 1) > 1e-6
    ):
        raise ValueError("Expected a finite rigid 4x4 transform")
    return transform


def pose_matrix(pose):
    """Convert [x, y, z, roll, pitch, yaw] using Rz(yaw) Ry(pitch) Rx(roll)."""
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("pose must contain six finite values (m, rad)")
    roll, pitch, yaw = values[3:]
    cr, sr, ct, st, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    transform = np.eye(4)
    transform[:3, :3] = [
        [cy * ct, cy * st * sr - sy * cr, cy * st * cr + sy * sr],
        [sy * ct, sy * st * sr + cy * cr, sy * st * cr - cy * sr],
        [-st, ct * sr, ct * cr],
    ]
    transform[:3, 3] = values[:3]
    return transform


def matrix_pose(value):
    """Convert a transform to metres and XYZ roll/pitch/yaw radians."""
    transform = matrix(value)
    rotation = transform[:3, :3]
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll, yaw = math.atan2(-rotation[1, 2], rotation[1, 1]), 0.0
    else:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return transform[:3, 3].tolist() + [roll, pitch, yaw]


def inverse(value):
    """Invert a rigid transform."""
    transform = matrix(value)
    output = np.eye(4)
    output[:3, :3] = transform[:3, :3].T
    output[:3, 3] = -output[:3, :3] @ transform[:3, 3]
    return output


def distance(first, second):
    """Return translation in metres and angular distance in degrees."""
    first, second = matrix(first), matrix(second)
    angle = math.acos(float(np.clip((np.trace(first[:3, :3].T @ second[:3, :3]) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(first[:3, 3] - second[:3, 3])), math.degrees(angle)


def average(transforms):
    """Average rotations on SO(3) and translations in Euclidean space."""
    if not transforms:
        raise ValueError("Need at least one transform")
    transforms = [matrix(value) for value in transforms]
    left, _, right = np.linalg.svd(sum(value[:3, :3] for value in transforms))
    output = np.eye(4)
    output[:3, :3] = left @ np.diag([1, 1, np.linalg.det(left @ right)]) @ right
    output[:3, 3] = np.mean([value[:3, 3] for value in transforms], axis=0)
    return output
