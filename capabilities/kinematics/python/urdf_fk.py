# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline fixed/revolute URDF chain, independent of any vendor connection."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def multiply(first, second):
    return [sum(first[row * 4 + k] * second[k * 4 + col] for k in range(4))
            for row in range(4) for col in range(4)]


def identity():
    return [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def validate_transform(pose):
    if len(pose) != 16 or not all(math.isfinite(v) for v in pose):
        raise ValueError("requires 16 finite transform entries")
    if any(abs(a-b) > 1e-8 for a, b in zip(pose[12:], [0, 0, 0, 1])):
        raise ValueError("invalid homogeneous row")
    for first in range(3):
        for second in range(3):
            dot = sum(pose[k*4+first] * pose[k*4+second] for k in range(3))
            if abs(dot - (1 if first == second else 0)) > 1e-6:
                raise ValueError("nonorthogonal rotation")
    determinant = (pose[0]*(pose[5]*pose[10]-pose[6]*pose[9]) -
                   pose[1]*(pose[4]*pose[10]-pose[6]*pose[8]) +
                   pose[2]*(pose[4]*pose[9]-pose[5]*pose[8]))
    if abs(determinant-1) > 1e-6:
        raise ValueError("improper rotation")


def rotation(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("invalid rotation axis")
    x, y, z = [value / length for value in axis]
    cosine, sine = math.cos(angle), math.sin(angle)
    one = 1 - cosine
    return [cosine+x*x*one, x*y*one-z*sine, x*z*one+y*sine, 0,
            y*x*one+z*sine, cosine+y*y*one, y*z*one-x*sine, 0,
            z*x*one-y*sine, z*y*one+x*sine, cosine+z*z*one, 0, 0, 0, 0, 1]


def vector(element, name, default):
    result = [float(value) for value in element.get(name, default).split()]
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("invalid URDF vector")
    return result


class UrdfKinematics:
    """Explicit base/tip and joint order; no mesh, collision or dynamics claims (IK: urdf_ik)."""

    def __init__(self, filename, base, tip, joint_names):
        if Path(filename).stat().st_size > 2 * 1024 * 1024:
            raise ValueError("URDF exceeds 2 MiB")
        robot = ET.parse(filename).getroot()
        by_child = {}
        for joint in robot.findall("joint"):
            child = joint.find("child").get("link")
            if child in by_child:
                raise ValueError("URDF has multiple parents")
            by_child[child] = joint
        chain, seen, current = [], set(), tip
        while current != base:
            if current in seen or current not in by_child:
                raise ValueError("missing or cyclic base-tip chain")
            seen.add(current)
            joint = by_child[current]
            chain.insert(0, joint)
            current = joint.find("parent").get("link")
        self.chain, self.names = chain, list(joint_names)
        active = [joint.get("name") for joint in chain if joint.get("type") != "fixed"]
        if active != self.names or any(joint.get("type") not in ("fixed", "revolute") for joint in chain):
            raise ValueError("unsupported chain or mismatched joint order")

    @property
    def limits(self):
        """URDF (lower, upper) limits of the active joints, in joint order."""
        result = []
        for joint in self.chain:
            if joint.get("type") == "revolute":
                limit = joint.find("limit")
                if limit is None:
                    raise ValueError("URDF joint limit missing")
                result.append((float(limit.get("lower")), float(limit.get("upper"))))
        return result

    def forward(self, joints_rad, check_limits=True):
        """Flange pose as a row-major 4x4 list. Measured joints may sit slightly past the URDF
        limits (the controller allows them), so pose measurement passes check_limits=False."""
        return self.frames(joints_rad, check_limits)[0]

    def frames(self, joints_rad, check_limits=True):
        """The tip pose and, per active joint, its origin and unit axis in the base frame."""
        if len(joints_rad) != len(self.names) or not all(math.isfinite(v) for v in joints_rad):
            raise ValueError("invalid joint vector")
        angles = dict(zip(self.names, joints_rad))
        pose = identity()
        joints = []
        for joint in self.chain:
            origin = joint.find("origin")
            offset = identity()
            if origin is not None:
                xyz = vector(origin, "xyz", "0 0 0")
                roll, pitch, yaw = vector(origin, "rpy", "0 0 0")
                offset = multiply(multiply(rotation([0, 0, 1], yaw), rotation([0, 1, 0], pitch)),
                                  rotation([1, 0, 0], roll))
                offset[3], offset[7], offset[11] = xyz
            pose = multiply(pose, offset)
            if joint.get("type") == "revolute":
                angle = angles[joint.get("name")]
                limit = joint.find("limit")
                if limit is None or (check_limits and
                                     not float(limit.get("lower")) <= angle <= float(limit.get("upper"))):
                    raise ValueError("URDF joint limit exceeded")
                axis = joint.find("axis")
                axis = vector(axis, "xyz", "1 0 0") if axis is not None else [1, 0, 0]
                length = math.sqrt(sum(value * value for value in axis))
                world = [sum(pose[row * 4 + k] * axis[k] for k in range(3)) / length for row in range(3)]
                joints.append(([pose[3], pose[7], pose[11]], world))
                pose = multiply(pose, rotation(axis, angle))
        return pose, joints
