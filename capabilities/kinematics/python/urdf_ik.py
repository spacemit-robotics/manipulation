# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Numerical inverse kinematics and Cartesian lines on a UrdfKinematics chain; no device access.

Damped least squares (damping 1e-4, step 0.5 halved down to 0.01, joint change at most 0.5 rad per
iteration, 0.5 mm, 200 iterations). Targets
are base-frame poses of the model tip. orientation_weight 0 solves position only: a five-joint arm
such as the SO101 cannot reach an arbitrary orientation.
"""
import math

import numpy as np

DAMPING = 1e-4
FIRST_STEP, LAST_STEP = 0.5, 0.01
MAXIMUM_JOINT_STEP_RAD = 0.5
POSITION_TOLERANCE_M = 5e-4
ORIENTATION_TOLERANCE_RAD = 0.01
MAXIMUM_ITERATIONS = 200
LINE_STEP_M = 0.005
LINE_MAXIMUM_POINTS = 400
LINE_MAXIMUM_JOINT_JUMP_RAD = 0.35  # Between consecutive waypoints: a larger jump changes IK branch.


def matrix(pose):
    """A 4x4 array from a row-major 16-element list or a 4x4 nested sequence."""
    result = np.asarray(pose, dtype=float).reshape(4, 4)
    if not np.isfinite(result).all():
        raise ValueError("pose must be finite")
    return result


def rpy_matrix(roll, pitch, yaw):
    """Rotation of URDF roll-pitch-yaw angles: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy(rotation):
    """URDF roll-pitch-yaw angles of a rotation matrix (pitch within [-pi/2, pi/2])."""
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) < 1e-9:  # Gimbal lock: put the whole yaw into roll.
        return [math.atan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.0]
    return [math.atan2(rotation[2, 1], rotation[2, 2]), pitch, math.atan2(rotation[1, 0], rotation[0, 0])]


def pose(position_m, rpy_rad=None, orientation=None):
    """A 4x4 pose from a position and roll-pitch-yaw angles (or a rotation matrix; default identity)."""
    result = np.eye(4)
    result[:3, 3] = np.asarray(position_m, dtype=float)
    if rpy_rad is not None:
        result[:3, :3] = rpy_matrix(*rpy_rad)
    elif orientation is not None:
        result[:3, :3] = np.asarray(orientation, dtype=float)
    return matrix(result)


def rotation_vector(rotation):
    """Axis times angle of a rotation matrix."""
    cosine = max(-1.0, min(1.0, (np.trace(rotation) - 1) / 2))
    angle = math.acos(cosine)
    skew = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-9:
        return skew / 2
    if math.pi - angle < 1e-6:  # Near pi the skew part vanishes: take the axis from the diagonal.
        axis = np.sqrt(np.maximum((np.diag(rotation) + 1) / 2, 0))
        index = int(np.argmax(axis))
        for other in range(3):
            if other != index:
                axis[other] = math.copysign(axis[other], rotation[index, other] + rotation[other, index])
        return axis / np.linalg.norm(axis) * angle
    return skew * angle / (2 * math.sin(angle))


def rotation_matrix(vector):
    """Rotation of an axis-times-angle vector (Rodrigues)."""
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = np.asarray(vector) / angle
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def jacobian(model, joints_rad):
    """Tip pose and the 6xN geometric Jacobian (linear rows first) in the base frame."""
    tip, frames = model.frames(list(joints_rad), check_limits=False)
    tip = matrix(tip)
    columns = []
    for origin, axis in frames:
        axis = np.asarray(axis)
        columns.append(np.concatenate([np.cross(axis, tip[:3, 3] - np.asarray(origin)), axis]))
    return tip, np.array(columns).T


def joint_limits(model, limits=None):
    """The model's URDF limits, narrowed by configured (lower, upper) limits when given."""
    result = [list(pair) for pair in model.limits]
    if limits is not None:
        if len(limits) != len(result):
            raise ValueError("configured limits do not match the model joints")
        for pair, (lower, upper) in zip(result, limits):
            pair[0], pair[1] = max(pair[0], float(lower)), min(pair[1], float(upper))
            if pair[0] > pair[1]:
                raise ValueError("configured and model joint limits do not overlap")
    return np.array(result)


def errors(model, joints_rad, target):
    """Position and orientation error of the tip to the target, and the Jacobian."""
    tip, columns = jacobian(model, joints_rad)
    position = target[:3, 3] - tip[:3, 3]
    orientation = rotation_vector(target[:3, :3] @ tip[:3, :3].T)
    return position, orientation, columns


def solve(model, target, seed_rad, *, orientation_weight=0.0, limits=None,
          position_tolerance_m=POSITION_TOLERANCE_M, orientation_tolerance_rad=ORIENTATION_TOLERANCE_RAD,
          maximum_iterations=MAXIMUM_ITERATIONS):
    """Joints reaching the target pose from seed_rad, within the model (and configured) limits.

    Returns a dict: joints_rad, converged, position_error_m, orientation_error_rad, iterations. A
    result that did not converge is the best found; callers must not move to it.
    """
    target = matrix(target)
    if not (math.isfinite(orientation_weight) and 0 <= orientation_weight <= 1):
        raise ValueError("orientation_weight must be in [0, 1]")
    bounds = joint_limits(model, limits)
    joints = np.clip(np.asarray(seed_rad, dtype=float), bounds[:, 0], bounds[:, 1])
    if joints.shape != (len(model.names),) or not np.isfinite(joints).all():
        raise ValueError("seed must hold one finite angle per model joint")

    def weighted(position, orientation):
        return np.concatenate([position, orientation_weight * orientation]) if orientation_weight else position

    def reached(position, orientation):
        return bool(np.linalg.norm(position) <= position_tolerance_m and (
            not orientation_weight or np.linalg.norm(orientation) <= orientation_tolerance_rad))

    position, orientation, columns = errors(model, joints, target)
    iterations = 0
    while iterations < maximum_iterations and not reached(position, orientation):
        jacobian_rows = (np.vstack([columns[:3], orientation_weight * columns[3:]]) if orientation_weight
                         else columns[:3])
        error = weighted(position, orientation)
        step = jacobian_rows.T @ np.linalg.solve(
            jacobian_rows @ jacobian_rows.T + DAMPING * np.eye(len(error)), error)
        largest = np.max(np.abs(step))
        if largest > MAXIMUM_JOINT_STEP_RAD:
            step *= MAXIMUM_JOINT_STEP_RAD / largest
        scale, improved = FIRST_STEP, False
        while scale >= LAST_STEP:
            candidate = np.clip(joints + scale * step, bounds[:, 0], bounds[:, 1])
            new_position, new_orientation, new_columns = errors(model, candidate, target)
            if np.linalg.norm(weighted(new_position, new_orientation)) < np.linalg.norm(error):
                joints, position, orientation, columns = candidate, new_position, new_orientation, new_columns
                improved = True
                break
            scale /= 2
        if not improved:  # Stalled: a joint limit or an unreachable target.
            break
        iterations += 1
    return dict(joints_rad=[float(value) for value in joints], converged=reached(position, orientation),
                position_error_m=float(np.linalg.norm(position)),
                orientation_error_rad=float(np.linalg.norm(orientation)), iterations=iterations)


def line(model, start_rad, target, *, step_m=LINE_STEP_M, orientation_weight=0.0, limits=None,
         maximum_points=LINE_MAXIMUM_POINTS):
    """Joint waypoints along a straight base-frame line from the tip pose at start_rad to target.

    The position is interpolated linearly and the orientation by spherical interpolation; every
    waypoint is solved from the previous one. One unreachable waypoint, or a joint jump that would
    change IK branch, rejects the whole line before any motion (as the former move_line did).
    """
    target = matrix(target)
    if not (math.isfinite(step_m) and step_m > 0):
        raise ValueError("step_m must be positive")
    start = matrix(model.forward(list(start_rad), check_limits=False))
    count = max(1, math.ceil(np.linalg.norm(target[:3, 3] - start[:3, 3]) / step_m))
    if count > maximum_points:
        raise ValueError(f"line needs {count} waypoints (at most {maximum_points}); use a larger step")
    turn = rotation_vector(start[:3, :3].T @ target[:3, :3])
    waypoints, joints = [], list(start_rad)
    for index in range(1, count + 1):
        fraction = index / count
        goal = pose(start[:3, 3] + fraction * (target[:3, 3] - start[:3, 3]),
                    orientation=start[:3, :3] @ rotation_matrix(fraction * turn))
        result = solve(model, goal, joints, orientation_weight=orientation_weight, limits=limits)
        if not result["converged"]:
            raise ValueError(f"waypoint {index}/{count} is unreachable "
                             f"(position error {result['position_error_m'] * 1000:.1f} mm)")
        if max(abs(a - b) for a, b in zip(result["joints_rad"], joints)) > LINE_MAXIMUM_JOINT_JUMP_RAD:
            raise ValueError(f"waypoint {index}/{count} changes IK branch; split the line or move by joints")
        joints = result["joints_rad"]
        waypoints.append(joints)
    return waypoints
