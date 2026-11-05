#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Bounded local device worker; no recipes, home, gripper or arbitrary SDK dispatch."""

import argparse
import bisect
import collections
import contextlib
import fcntl
import importlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import sys
import threading
import time

SDK_COMMIT = "e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b"
MAX_FRAME = 1 << 20  # Timed trajectories carry their knots.
HOLD_SPEED_PERCENT = 5
# A seven-axis target spans several CAN frames; one may not take effect, and the controller may
# stop short of a target. A target is re-sent unchanged (never interpolated) when the arm has
# stood still off target, beyond the command's tolerance, for RESEND_INTERVAL_S.
RESEND_INTERVAL_S = 1.0
RESEND_LIMIT = 5
# `moving` is measured from joint feedback (200 Hz): any joint changing by more than STILL_RAD
# within MOVING_WINDOW_S. The controller's motion_status only means "target not yet reached":
# it can stay set while the arm stands still short of a target.
MOVING_WINDOW_S = 0.2
STILL_RAD = math.radians(0.05)
HISTORY_GAP_S = 0.05
# Joint feedback is recorded in the background, so a read after an idle period never waits for
# the window to fill.
RECORD_PERIOD_S = 0.01
# A point may ask for the original controller-endpoint delivery instead: the same target re-sent
# at a fixed interval while beyond tolerance, whether or not the arm is moving.
RESEND_INTERVAL_RANGE_S = (0.02, 0.2)
# Modes are switched explicitly, only on change; joint targets sent right after a switch can be
# ignored, so a short pause follows each switch.
MODE_SETTLE_S = 0.1
# Continuous paths: a quintic joint-space profile streamed as small move_js steps (the controller
# does no smoothing in this mode). Any late tick, large step or stale feedback stops the stream.
STREAM_HZ = 50
STREAM_MAX_STEP_RAD = math.radians(0.5)
STREAM_MAX_GAP_S = 0.1
STREAM_FEEDBACK_TIMEOUT_S = 1.0
PATH_MAX_WAYPOINTS = 128
# Timed trajectories (application-planned profiles such as a shake): bounds as the session's.
TRAJECTORY_MAX_KNOTS = 4000
TRAJECTORY_MAX_STEP_RAD = math.radians(3)
TRAJECTORY_START_RAD = math.radians(0.5)
TRAJECTORY_MAX_GAP_S = 0.3
ENVELOPE_MARGIN_DEG = 5.0
PATH_MAX_VELOCITY = 0.35  # rad/s; 0.4 deg per 20 ms step is about 20 deg/s
PATH_MAX_ACCELERATION = 1.0  # rad/s^2
TOLERANCE_MAX_RAD = math.radians(
    0.8
)  # js mode leaves a few tenths of a degree of steady-state error


def check_socketcan_state(channel, interfaces=Path("/sys/class/net")):
    """Refuse a known-down/non-CAN host interface without opening or configuring the bus.

    Sysfs may be unavailable (and CAN-free SDK fixtures have no host interface); in that case
    opening the channel and validating complete, fresh feedback remain the SDK's responsibility.
    CAN devices can report operstate UNKNOWN while up, so inspect IFF_UP, not operstate.
    """
    interface = interfaces / channel
    if not interface.exists():
        return
    if int((interface / "type").read_text().strip()) != 280:  # ARPHRD_CAN
        raise RuntimeError(f"SocketCAN {channel} is not a CAN interface")
    if not int((interface / "flags").read_text().strip(), 0) & 1:  # IFF_UP
        raise RuntimeError(
            f"SocketCAN {channel} is DOWN; stop control programs and have the operator "
            "check CAN wiring, power and bitrate before bringing the interface UP"
        )


def path_profile(points, velocity, acceleration):
    """Polyline (largest joint change per segment) with quintic time scaling: zero speed at both
    ends."""
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + max(abs(b - a) for a, b in zip(first, second)))
    distance = cumulative[-1]
    # Peak normalised velocity 1.875 and acceleration 5.774 of 10u^3 - 15u^4 + 6u^5.
    duration = max(0.3, 1.875 * distance / velocity, math.sqrt(5.774 * distance / acceleration))
    return points, cumulative, duration


def path_target(profile, elapsed):
    points, cumulative, duration = profile
    u = min(1.0, max(0.0, elapsed / duration))
    distance = (10 * u**3 - 15 * u**4 + 6 * u**5) * cumulative[-1]
    if distance >= cumulative[-1]:
        return list(points[-1])
    i = min(len(points) - 2, bisect.bisect_right(cumulative, distance) - 1)
    span = cumulative[i + 1] - cumulative[i]
    fraction = 0.0 if span < 1e-12 else (distance - cumulative[i]) / span
    return [a + fraction * (b - a) for a, b in zip(points[i], points[i + 1])]


class Stream(threading.Thread):
    """Sends sample(elapsed) with move_js at `rate` Hz until `duration`, then stops.

    A late tick never skips ahead: elapsed time advances at most `advance` per tick, so the motion
    only takes longer. Any pause beyond max_gap, commanded step beyond max_step, stale feedback or
    feedback outside `envelope` stops the stream; it then retargets the latest feedback and
    records why. The owner's next poll reports the error.
    """

    def __init__(
        self,
        connection,
        sample,
        duration,
        *,
        rate,
        max_step,
        max_gap,
        advance,
        final,
        tolerance,
        envelope=None,
        name="agx-stream",
    ):
        super().__init__(name=name, daemon=True)
        self.connection, self.sample, self.duration = connection, sample, duration
        self.rate, self.max_step, self.max_gap, self.advance = rate, max_step, max_gap, advance
        self.final, self.tolerance, self.envelope = list(final), tolerance, envelope
        self.stop = threading.Event()
        self.error = None
        self.done = False

    def run(self):
        connection, robot = self.connection, self.connection.robot
        last = self.sample(0.0)
        begin = previous = last_tick = last_feedback = time.monotonic()
        elapsed, tick, stamp = 0.0, 0, None
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if tick and now - previous > self.max_gap:
                    raise RuntimeError(f"stream paused {1000 * (now - previous):.0f} ms")
                elapsed = min(self.duration, elapsed + min(max(0.0, now - last_tick), self.advance))
                last_tick = now
                target = self.sample(elapsed)
                if max(abs(a - b) for a, b in zip(target, last)) > self.max_step + 1e-9:
                    raise RuntimeError(f"stream step exceeds {math.degrees(self.max_step):.2f} deg")
                feedback = robot.get_joint_angles()
                if feedback is not None and feedback.timestamp != stamp:
                    stamp, last_feedback = feedback.timestamp, now
                    if self.envelope is not None and any(
                        not low <= q <= high for q, (low, high) in zip(feedback.msg, self.envelope)
                    ):
                        raise RuntimeError("actual joint angle outside the planned envelope")
                if now - last_feedback > STREAM_FEEDBACK_TIMEOUT_S:
                    raise RuntimeError("joint feedback stale during stream")
                with connection.sdk_lock:
                    if self.stop.is_set():
                        break
                    robot.move_js(target)
                last, previous = target, time.monotonic()
                if elapsed >= self.duration:
                    taken = time.monotonic() - begin
                    print(
                        f"agx_worker: {self.name} planned {self.duration:.2f} s, "
                        f"took {taken:.2f} s, {tick + 1} commands "
                        f"({(tick + 1) / max(taken, 1e-3):.0f} Hz of {self.rate:.0f} Hz)",
                        file=sys.stderr,
                        flush=True,
                    )
                    self.done = True
                    break
                tick += 1
                self.stop.wait(max(0.0, begin + tick / self.rate - time.monotonic()))
        except Exception as error:  # noqa: BLE001 - reported through the owner's next poll
            self.error = f"{type(error).__name__}: {error}"
            with contextlib.suppress(Exception), connection.sdk_lock:
                feedback = robot.get_joint_angles()
                robot.move_js(list(feedback.msg) if feedback is not None else last)


def knot_sampler(times, positions):
    """Linear interpolation between timed knots."""

    def sample(elapsed):
        i = min(len(times) - 2, max(0, bisect.bisect_right(times, elapsed) - 1))
        span = times[i + 1] - times[i]
        fraction = min(1.0, max(0.0, (elapsed - times[i]) / span))
        return [a + fraction * (b - a) for a, b in zip(positions[i], positions[i + 1])]

    return sample


class AgxConnection:
    """One owner for the whole CAN channel, including the legacy cooperative lease.

    `wrote` records whether the current request sent anything to the device. A failure
    before any write is reported as rejected; the owner may keep the connection.
    """

    def __init__(self, sdk_root, channel):
        self.sdk_root = Path(sdk_root).resolve()
        self.channel = channel
        self.robot = None
        self.lease = None
        self.firmware = "unknown"
        self.sequence = 0
        self.last_stamps = None
        self.hand = None
        self.hand_feedback = "position"
        self.hand_sequence = 0
        self.hand_stamps = None
        self.speed_percent = HOLD_SPEED_PERCENT
        self.wrote = False
        self.pending = None  # The target of the running move, until reached or held.
        self.mode = None  # Motion mode last set on the controller: "j" or "js".
        self.history = collections.deque()  # (monotonic s, joints) from distinct joint frames.
        self.joint_stamp = None
        self.history_lock = threading.Lock()  # The recorder and requests both append.
        self.recorder = None
        self.recording = threading.Event()
        self.controller_limits = None
        self.stream = None  # The running path stream, until it finishes, fails or is held.
        self.sdk_lock = (
            threading.Lock()
        )  # SDK writes from requests and the stream never interleave.

    def write(self, operation, *args, **kwargs):
        self.wrote = True
        with self.sdk_lock:
            return operation(*args, **kwargs)

    def connect(self):
        self.wrote = True  # A failed connection is never reused.
        if self.robot is not None:
            raise RuntimeError("already connected")
        marker = self.sdk_root / "UPSTREAM_COMMIT"
        if marker.read_text(encoding="utf-8").strip() != SDK_COMMIT:
            raise RuntimeError("pyAgxArm revision mismatch")
        check_socketcan_state(self.channel)
        path = f"/tmp/nero_{self.channel}_control.lock"
        self.lease = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        if not stat.S_ISREG(os.fstat(self.lease).st_mode):
            raise RuntimeError("invalid legacy lease")
        fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sys.path.insert(0, str(self.sdk_root))
        sdk = importlib.import_module("pyAgxArm")
        if not Path(sdk.__file__).resolve().is_relative_to(self.sdk_root):
            raise RuntimeError("SDK imported outside pinned source root")
        config = sdk.create_agx_arm_config(
            robot=sdk.ArmModel.NERO,
            firmeware_version=sdk.NeroFW.V120,
            interface="socketcan",
            channel=self.channel,
        )
        self.robot = sdk.AgxArmFactory.create_arm(config)
        self.robot.set_joint_limits_enabled(True)  # SDK-side clamp to the vendor preset, as before.
        self.robot.set_auto_set_motion_mode_enabled(False)  # Modes are set explicitly (set_mode).
        self.robot.connect()  # No enable, mode switch, home, reset or effector construction.
        self.recording.set()
        self.recorder = threading.Thread(target=self.record, name="agx-feedback", daemon=True)
        self.recorder.start()
        deadline = time.monotonic() + 1.5
        while True:
            try:
                return self.read()
            except RuntimeError as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"SocketCAN {self.channel}: {error}; check interface state, bitrate, "
                        "device power and complete feedback"
                    ) from error
                time.sleep(0.01)

    def close(self):
        self.pending = None
        with contextlib.suppress(RuntimeError):
            self.stop_stream()
        self.recording.clear()
        if self.recorder is not None:
            self.recorder.join(timeout=0.5)
            self.recorder = None
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None
            self.hand = None
        if self.lease is not None:
            os.close(self.lease)
            self.lease = None

    def record(self):
        """Keep the joint history covering MOVING_WINDOW_S between requests (read-only)."""
        while self.recording.is_set():
            robot = self.robot
            with contextlib.suppress(Exception):  # A read failure surfaces in the next request.
                parser = robot._parser  # pylint: disable=protected-access
                parts = [
                    getattr(parser, name)
                    for name in ("joint_12", "joint_34", "joint_56", "joint_7")
                ]
                stamps = tuple(float(part.timestamp) for part in parts)
                angles = [
                    float(getattr(part.msg, f"joint_{index}"))
                    for part, indices in zip(parts, ((1, 2), (3, 4), (5, 6), (7,)))
                    for index in indices
                ]
                if stamps == tuple(float(part.timestamp) for part in parts) and all(
                    map(math.isfinite, angles)
                ):
                    self.remember(max(stamps), angles, time.time(), time.monotonic())
            time.sleep(RECORD_PERIOD_S)

    def remember(self, joint_stamp, angles, wall_now, mono_now):
        with self.history_lock:
            if self.joint_stamp is None or joint_stamp > self.joint_stamp:
                self.joint_stamp = joint_stamp
                self.history.append((mono_now - (wall_now - joint_stamp), angles))

    def read(self):
        # Right after connecting the joint history is too short to tell motion: sample briefly.
        deadline = time.monotonic() + MOVING_WINDOW_S + 0.2
        while True:
            state = self.sample()
            if state["moving"] is not None:
                return state
            if time.monotonic() >= deadline:
                return dict(state, moving=state.pop("controller_moving"))
            time.sleep(0.005)

    def sample(self):
        for _attempt in range(3):
            try:
                return self.snapshot()
            except RuntimeError as error:
                if str(error) != "feedback changed during snapshot; retry required":
                    raise
        raise RuntimeError("feedback snapshot raced repeatedly")

    def measured_motion(self, now):
        """True/False from a continuous joint history covering MOVING_WINDOW_S, else None."""
        with self.history_lock:
            while self.history and self.history[0][0] < now - 1.0:
                self.history.popleft()
            recent = [(t, q) for t, q in self.history if t >= now - MOVING_WINDOW_S - HISTORY_GAP_S]
        if (
            not recent
            or recent[0][0] > now - MOVING_WINDOW_S
            or now - recent[-1][0] > HISTORY_GAP_S
            or any(b[0] - a[0] > HISTORY_GAP_S for a, b in zip(recent, recent[1:]))
        ):
            return None
        return any(max(column) - min(column) > STILL_RAD for column in zip(*(q for _, q in recent)))

    def snapshot(self):
        if self.robot is None:
            raise RuntimeError("not connected")
        # Pinned SDK get_joint_angles() alone timestamps only the last existing part.
        # Inspect all four parser frames, isolated here until upstream exposes completeness.
        parser = self.robot._parser  # pylint: disable=protected-access
        parts = [
            getattr(parser, name, None) for name in ("joint_12", "joint_34", "joint_56", "joint_7")
        ]
        status = self.robot.get_arm_status()
        motors = [self.robot.get_driver_states(i) for i in range(1, 8)]
        messages = parts + [status] + motors
        if any(item is None for item in messages):
            raise RuntimeError("incomplete joint/status/enable feedback")
        before = tuple(float(item.timestamp) for item in messages)
        angles = [
            float(getattr(part.msg, f"joint_{index}"))
            for part, indices in zip(parts, ((1, 2), (3, 4), (5, 6), (7,)))
            for index in indices
        ]
        enabled = all(bool(motor.msg.foc_status.driver_enable_status) for motor in motors)
        fault = (
            int(status.msg.arm_status) != 0
            or int(status.msg.err_code) != 0
            or any(int(motor.msg.foc_status_code) & 0xBF for motor in motors)
        )  # Bit 6 is enable; every other motor status bit is an abnormal condition.
        position_mode = int(status.msg.ctrl_mode) == 1 and int(status.msg.mode_feedback) in (
            0,
            1,
            2,
            3,
        )
        motion = int(status.msg.motion_status)
        after = tuple(float(item.timestamp) for item in messages)
        wall_now, mono_now = time.time(), time.monotonic_ns()
        if before != after:
            raise RuntimeError("feedback changed during snapshot; retry required")
        if not all(math.isfinite(value) for value in angles + list(before)):
            raise RuntimeError("nonfinite feedback")
        oldest, newest = min(before), max(before)
        age = wall_now - oldest
        if oldest <= 0 or age < 0 or age > 0.2 or newest - oldest > 0.1:
            raise RuntimeError("stale or incoherent feedback")
        if motion not in (0, 1):
            raise RuntimeError("unknown motion state")
        # Advance only when EVERY constituent frame has advanced, not when polled again.
        if self.last_stamps is None or all(a > b for a, b in zip(before, self.last_stamps)):
            self.sequence += 1
            self.last_stamps = before
        self.remember(max(before[:4]), angles, wall_now, mono_now / 1e9)
        return {
            "joints_rad": angles,
            "source_time_s": oldest,
            "received_ns": mono_now - int(age * 1e9),
            "sample_id": self.sequence,
            "valid": True,
            "enabled": enabled,
            "position_mode": position_mode,
            "moving": self.measured_motion(mono_now / 1e9),
            "controller_moving": motion == 1,
            "fault": fault,
            "firmware": self.firmware,
            "command_active": False,
            **({"controller_limits": self.controller_limits} if self.controller_limits else {}),
        }

    def diagnose(self):
        result = self.robot.get_firmware(timeout=1.0)
        self.firmware = str((result or {}).get("software_version", "unknown"))
        if self.firmware != "1.20":
            raise RuntimeError(f"expected firmware 1.20, received {self.firmware}")
        self.controller_limits = self.read_controller_limits()
        return self.read()

    def read_controller_limits(self):
        """The controller's live joint position/velocity/acceleration limits (read-only queries)."""
        limits = []
        for joint in range(1, 8):
            angle = self.robot.get_joint_angle_vel_limits(joint, timeout=1.0, min_interval=0.0)
            acceleration = self.robot.get_joint_acc_limits(joint, timeout=1.0, min_interval=0.0)
            if angle is None or acceleration is None:
                return None  # Unknown: applications that plan with them must refuse.
            row = {
                "joint": joint,
                "min_angle_rad": float(angle.msg.min_angle_limit),
                "max_angle_rad": float(angle.msg.max_angle_limit),
                "max_velocity_rad_s": float(angle.msg.max_joint_spd),
                "max_acceleration_rad_s2": float(acceleration.msg.max_joint_acc),
            }
            if (
                not all(math.isfinite(v) for v in row.values())
                or row["min_angle_rad"] >= row["max_angle_rad"]
                or row["max_velocity_rad_s"] <= 0
                or row["max_acceleration_rad_s2"] <= 0
            ):
                return None
            limits.append(row)
        return limits

    def ready(self, idle=True):
        """Healthy, and (idle) not still streaming a command. Whether the arm may start a command
        while it settles after one that did not wait for arrival is the owner's decision, not this
        one's."""
        if idle and self.stream is not None:
            raise RuntimeError("a stream is still running")
        state = self.read()
        if (
            self.firmware != "1.20"
            or state["fault"]
            or not state["enabled"]
            or not state["position_mode"]
        ):
            raise RuntimeError("diagnose firmware, fault, enable and CAN mode before motion")
        return state

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = None  # Unknown until the switch has been written.
            self.write(self.robot.set_motion_mode, mode)
            self.mode = mode
            time.sleep(MODE_SETTLE_S)

    def enable(self):
        if self.stream is not None:
            raise RuntimeError("a path stream is still running")
        state = self.read()
        if self.firmware != "1.20" or state["fault"] or state["moving"]:
            raise RuntimeError("diagnose firmware and stationary normal state before enabling")
        if not self.write(self.robot.enable, timeout=1.0):
            raise RuntimeError("enable not confirmed")
        return self.read()

    @staticmethod
    def at(state, target_rad, tolerance):
        """Already resting on the target: sending it again would only re-plan, maybe switching
        the control mode."""
        return (
            not state["moving"]
            and max(abs(a - b) for a, b in zip(state["joints_rad"], target_rad)) <= tolerance
        )

    @staticmethod
    def check_tolerance(tolerance):
        if not (
            isinstance(tolerance, (int, float))
            and math.isfinite(tolerance)
            and 0 < tolerance <= TOLERANCE_MAX_RAD
        ):
            raise RuntimeError("tolerance_rad must be in (0, 0.8 deg]")

    def submit(self, target_rad, speed_scale, tolerance, resend_interval=0.0):
        state = self.ready()
        if len(target_rad) != 7 or not all(math.isfinite(q) for q in target_rad):
            raise RuntimeError("seven finite joint targets required")
        if not math.isfinite(speed_scale) or not 0.01 <= speed_scale <= 1:
            raise RuntimeError("Nero speed_scale must be 0.01..1 (integer controller percent)")
        self.check_tolerance(tolerance)
        low, high = RESEND_INTERVAL_RANGE_S
        if not (
            isinstance(resend_interval, (int, float))
            and math.isfinite(resend_interval)
            and (resend_interval == 0 or low <= resend_interval <= high)
        ):
            raise RuntimeError("resend_interval_s must be 0 or 0.02..0.2")
        self.pending = None
        if self.at(state, target_rad, tolerance):
            return {"accepted": True, "sent": False}
        self.speed_percent = max(1, math.floor(speed_scale * 100))
        self.write(self.robot.set_speed_percent, self.speed_percent)
        self.set_mode("j")
        self.write(self.robot.move_j, list(target_rad))  # PTP, never unsmoothed move_js.
        self.track(target_rad, tolerance, state, resend_interval)
        return {"accepted": True}

    def track(self, target_rad, tolerance, state, resend_interval=0.0):
        error = max(abs(a - b) for a, b in zip(state["joints_rad"], target_rad))
        now = time.monotonic()
        self.pending = {
            "target": list(target_rad),
            "tolerance": tolerance,
            "resends": 0,
            "best": error,
            "progress": now,
            "sent": now,
            "interval": resend_interval,
        }

    def submit_path(self, path_rad, speed_scale, velocity, acceleration, tolerance):
        state = self.ready()
        if (
            not isinstance(path_rad, list)
            or not 1 <= len(path_rad) <= PATH_MAX_WAYPOINTS
            or any(
                not isinstance(q, list) or len(q) != 7 or not all(math.isfinite(v) for v in q)
                for q in path_rad
            )
        ):
            raise RuntimeError("path needs 1..128 waypoints of seven finite joints")
        if not math.isfinite(speed_scale) or not 0.01 <= speed_scale <= 1:
            raise RuntimeError("Nero speed_scale must be 0.01..1 (integer controller percent)")
        if not (
            math.isfinite(velocity)
            and 0 < velocity <= PATH_MAX_VELOCITY
            and math.isfinite(acceleration)
            and 0 < acceleration <= PATH_MAX_ACCELERATION
        ):
            raise RuntimeError("path velocity/acceleration out of range")
        self.check_tolerance(tolerance)
        profile = path_profile(
            [state["joints_rad"]] + [list(q) for q in path_rad], velocity, acceleration
        )
        self.pending = None
        if all(self.at(state, q, tolerance) for q in path_rad):
            return {"accepted": True, "sent": False, "duration_s": 0.0}
        return self.start_stream(
            Stream(
                self,
                lambda elapsed: path_target(profile, elapsed),
                profile[2],
                rate=STREAM_HZ,
                max_step=STREAM_MAX_STEP_RAD,
                max_gap=STREAM_MAX_GAP_S,
                advance=STREAM_MAX_STEP_RAD * 0.8 / velocity,
                final=profile[0][-1],
                tolerance=tolerance,
                name="agx-path-stream",
            ),
            speed_scale,
        )

    def submit_trajectory(self, times, positions, rate, max_step, speed_scale, tolerance):
        state = self.ready()
        if (
            not isinstance(times, list)
            or not isinstance(positions, list)
            or len(times) != len(positions)
            or not 2 <= len(times) <= TRAJECTORY_MAX_KNOTS
            or any(
                not isinstance(q, list) or len(q) != 7 or not all(math.isfinite(v) for v in q)
                for q in positions
            )
            or times[0] != 0
            or any(not b > a for a, b in zip(times, times[1:]))
            or times[-1] > 60
        ):
            raise RuntimeError(
                "trajectory needs 2..4000 increasing knots of seven finite joints, at most 60 s"
            )
        if not (
            math.isfinite(rate)
            and 20 <= rate <= 500
            and math.isfinite(max_step)
            and 0 < max_step <= TRAJECTORY_MAX_STEP_RAD * (1 + 1e-9)
        ):
            raise RuntimeError("command_rate_hz must be 20..500 and max_step_rad (0, 3 deg]")
        if not math.isfinite(speed_scale) or not 0.01 <= speed_scale <= 1:
            raise RuntimeError("Nero speed_scale must be 0.01..1 (integer controller percent)")
        self.check_tolerance(tolerance)
        if (
            max(abs(a - b) for a, b in zip(state["joints_rad"], positions[0]))
            > TRAJECTORY_START_RAD
        ):
            raise RuntimeError("trajectory must start at the current position (0.5 deg)")
        margin = math.radians(ENVELOPE_MARGIN_DEG)
        envelope = [
            (min(column) - margin, max(column) + margin)
            for column in zip(state["joints_rad"], *positions)
        ]
        # A late tick advances time at most so far that the step stays within 0.9 max_step (as the
        # original max_phase_step): the motion runs slower instead of jumping.
        peak = max(
            max(abs(b - a) for a, b in zip(p, q)) / (t2 - t1)
            for t1, t2, p, q in zip(times, times[1:], positions, positions[1:])
        )
        advance = min(2 / rate, 0.9 * max_step / peak) if peak > 0 else 2 / rate
        self.pending = None
        return self.start_stream(
            Stream(
                self,
                knot_sampler(times, positions),
                times[-1],
                rate=rate,
                max_step=max_step,
                max_gap=TRAJECTORY_MAX_GAP_S,
                advance=advance,
                final=positions[-1],
                tolerance=tolerance,
                envelope=envelope,
                name="agx-trajectory-stream",
            ),
            speed_scale,
        )

    def start_stream(self, stream, speed_scale):
        self.speed_percent = max(1, math.floor(speed_scale * 100))
        self.write(self.robot.set_speed_percent, self.speed_percent)
        self.set_mode("js")  # Once; each step then sends only targets.
        self.stream = stream
        stream.start()
        return {"accepted": True, "duration_s": stream.duration}

    def stop_stream(self):
        stream, self.stream = self.stream, None
        if stream is not None:
            stream.stop.set()
            stream.join(timeout=1.0)
            if stream.is_alive():
                raise RuntimeError("path stream did not stop")

    def poll(self):
        """Feedback for the owner's progress polling; re-sends a target the controller dropped."""
        state = self.read()
        stream = self.stream
        if stream is not None:
            if stream.error:
                raise RuntimeError("path stream stopped: " + stream.error)
            if (
                not stream.done
            ):  # Never judged settled, nor delivered, while targets are still being sent.
                return dict(state, moving=True, command_active=True)
            self.stop_stream()
            self.track(stream.final, stream.tolerance, state)  # Settles in js mode, as streamed.
            return dict(state, moving=True)  # This sample predates the last streamed target.
        pending = self.pending
        if pending is None:
            return state
        error = max(abs(a - b) for a, b in zip(state["joints_rad"], pending["target"]))
        now = time.monotonic()
        if error < pending["best"] - STILL_RAD:
            pending["best"], pending["progress"] = error, now
        if (
            error <= pending["tolerance"]
            or state["fault"]
            or not state["enabled"]
            or not state["position_mode"]
        ):
            self.pending = None  # Reached, or no longer ours to drive; the owner judges the result.
        elif pending["interval"]:
            if now - pending["sent"] >= pending["interval"]:  # Bounded by the owner's deadline.
                pending["sent"] = now
                self.write(
                    self.robot.move_js if self.mode == "js" else self.robot.move_j,
                    pending["target"],
                )
        elif (
            state["moving"] is False
            and pending["resends"] < RESEND_LIMIT
            and now - max(pending["progress"], pending["sent"]) >= RESEND_INTERVAL_S
        ):
            pending["resends"] += 1
            pending["sent"] = now
            print(
                f"agx_worker: arm still {error:.4f} rad from target ({self.mode} mode); "
                f"re-sent the same target ({pending['resends']}/{RESEND_LIMIT})",
                file=sys.stderr,
                flush=True,
            )
            self.write(
                self.robot.move_js if self.mode == "js" else self.robot.move_j, pending["target"]
            )
        return state

    def hold(self):
        # Retarget to the sampled position, never faster than the motion being stopped.
        self.pending = None
        self.stop_stream()  # Raises if the stream cannot be stopped: the hold is then not verified.
        state = self.ready(idle=False)
        self.write(self.robot.set_speed_percent, min(HOLD_SPEED_PERCENT, self.speed_percent))
        self.set_mode("j")
        state = self.ready(idle=False)  # The pose after the mode switch pause.
        self.write(self.robot.move_j, state["joints_rad"])
        deadline = time.monotonic() + 1.0
        previous = None
        stable_since = None
        count = 0
        while time.monotonic() < deadline:
            current = self.ready(idle=False)
            if previous and current["sample_id"] > previous["sample_id"]:
                stable = (
                    not current["moving"]
                    and max(
                        abs(a - b) for a, b in zip(current["joints_rad"], previous["joints_rad"])
                    )
                    <= 0.001
                )
                stable_since = (stable_since or time.monotonic()) if stable else None
                count = count + 1 if stable else 0
                if count >= 3 and stable_since and time.monotonic() - stable_since >= 0.1:
                    return {"held": True}
            previous = current
            time.sleep(0.01)
        return {"held": False}

    def effector_connect(self, feedback="position"):
        if self.robot is None:
            raise RuntimeError("arm connection required before effector")
        if feedback not in ("position", "none"):
            raise RuntimeError("effector feedback must be position or none")
        self.hand_feedback = feedback
        if self.hand is None:
            self.hand = self.write(self.robot.init_effector, self.robot.OPTIONS.EFFECTOR.REVO2)
        deadline = time.monotonic() + 1.5
        while True:
            try:
                return self.effector_read()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)

    def effector_liveness(self):
        """No position/status frames: a fresh finger-current frame (0x1C3) proves presence only."""
        current = self.hand.get_finger_current()
        if current is None:
            raise RuntimeError("no Revo2 current frames; hand absent or unpowered")
        stamp = float(current.timestamp)
        age = time.time() - stamp
        if not math.isfinite(stamp) or not 0 <= age <= 0.2:
            raise RuntimeError("stale Revo2 current feedback")
        if self.hand_stamps is None or stamp > self.hand_stamps[0]:
            self.hand_sequence += 1
            self.hand_stamps = (stamp,)
        return {
            "positions": [],
            "sample_id": self.hand_sequence,
            "received_ns": time.monotonic_ns() - int(age * 1e9),
            "valid": True,
            "moving": False,
            "fault": False,
            "side": "unknown",
        }

    def effector_read(self):
        if self.hand is None:
            raise RuntimeError("effector not connected")
        if self.hand_feedback == "none":
            return self.effector_liveness()
        names = (
            "thumb_tip",
            "thumb_base",
            "index_finger",
            "middle_finger",
            "ring_finger",
            "pinky_finger",
        )
        position, status = self.hand.get_finger_pos(), self.hand.get_hand_status()
        if position is None or status is None:
            raise RuntimeError("incomplete Revo2 position/status feedback")
        stamps = tuple(float(item.timestamp) for item in (position, status))
        values = [float(getattr(position.msg, name)) for name in names]
        modes = [int(getattr(status.msg, name)) for name in names]
        if int(status.msg.left_or_right) != 2:
            raise RuntimeError("right Revo2 feedback required; missing/left side rejected")
        now = time.time()
        age = now - min(stamps)
        if (
            not all(math.isfinite(x) for x in values + list(stamps))
            or not all(0 <= now - stamp <= 0.2 for stamp in stamps)
            or max(stamps) - min(stamps) > 0.1
        ):
            raise RuntimeError("stale/nonfinite Revo2 feedback")
        # Commands are 0..100; position feedback is 0..255 (measured: command 30 -> feedback 77).
        if any(not 0 <= value <= 255 for value in values) or any(
            mode not in (0, 1, 2) for mode in modes
        ):
            raise RuntimeError("invalid Revo2 feedback value/status")
        if self.hand_stamps is None or all(a > b for a, b in zip(stamps, self.hand_stamps)):
            self.hand_sequence += 1
            self.hand_stamps = stamps
        return {
            "positions": [value / 255 for value in values],
            "sample_id": self.hand_sequence,
            "received_ns": time.monotonic_ns() - int(age * 1e9),
            "valid": True,
            "moving": any(mode == 1 for mode in modes),
            "fault": any(mode == 2 for mode in modes),
            "side": "right",
        }

    def effector_submit(self, positions, duration_s):
        self.ready(idle=False)  # A gesture sends hand targets while the arm streams.
        current = self.effector_read()
        if current["fault"] or current["moving"]:
            raise RuntimeError("effector faulted or moving")
        if len(positions) != 6 or not all(math.isfinite(x) and 0 <= x <= 1 for x in positions):
            raise RuntimeError("six normalized effector targets required")
        if not math.isfinite(duration_s) or not 0 < duration_s <= 2.55:
            raise RuntimeError("effector duration must be (0,2.55] seconds")
        names = (
            "thumb_tip",
            "thumb_base",
            "index_finger",
            "middle_finger",
            "ring_finger",
            "pinky_finger",
        )
        target = {name: math.floor(value * 100 + 0.5) for name, value in zip(names, positions)}
        ticks = max(1, math.ceil(duration_s * 100))
        started = time.monotonic()
        self.write(self.hand.position_time_ctrl, mode="pos", **target)
        self.write(self.hand.position_time_ctrl, mode="time", **dict.fromkeys(names, ticks))
        if time.monotonic() - started > 0.05:
            raise RuntimeError("Revo2 paired frame deadline exceeded; target execution unknown")
        return {"accepted": True}


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate protocol key")
        result[key] = value
    return result


def serve(connection, reader, writer):
    session = None
    last_id = 0
    while True:
        line = reader.readline(MAX_FRAME + 1)
        if not line:
            return
        if len(line) > MAX_FRAME or not line.endswith(b"\n"):
            raise RuntimeError("oversized or truncated request")
        request = json.loads(line, object_pairs_hook=unique_object)
        if request.get("v") != 1 or not isinstance(request.get("session"), str):
            raise RuntimeError("invalid protocol version/session")
        session = session or request["session"]
        if (
            request["session"] != session
            or type(request.get("id")) is not int
            or request["id"] <= last_id
        ):
            raise RuntimeError("request identity replay or session mismatch")
        last_id = request["id"]
        response = {"v": 1, "session": session, "id": last_id, "ok": False}
        connection.wrote = False
        try:
            if (
                type(request.get("deadline_ns")) is not int
                or time.monotonic_ns() >= request["deadline_ns"]
            ):
                raise RuntimeError("request deadline exceeded before dispatch")
            operation = request.get("op")
            allowed = {"v", "session", "id", "deadline_ns", "op"}
            if operation == "submit":
                allowed |= {"target_rad", "speed_scale", "tolerance_rad", "resend_interval_s"}
            if operation == "submit_path":
                allowed |= {
                    "path_rad",
                    "speed_scale",
                    "max_velocity_rad_s",
                    "max_acceleration_rad_s2",
                    "tolerance_rad",
                }
            if operation == "submit_trajectory":
                allowed |= {
                    "times_s",
                    "positions_rad",
                    "command_rate_hz",
                    "max_step_rad",
                    "speed_scale",
                    "tolerance_rad",
                }
            if operation == "effector_submit":
                allowed |= {"positions", "duration_s"}
            if operation == "effector_connect":
                allowed |= {"feedback"}
            if set(request) - allowed:
                raise RuntimeError("unknown request fields")
            dispatch = {
                "connect": connection.connect,
                "read": connection.poll,
                "diagnose": connection.diagnose,
                "enable": connection.enable,
                "hold": connection.hold,
            }
            if operation == "effector_read":
                dispatch[operation] = connection.effector_read
            with contextlib.redirect_stdout(sys.stderr):
                if operation == "submit":
                    result = connection.submit(
                        request["target_rad"],
                        request["speed_scale"],
                        request["tolerance_rad"],
                        request.get("resend_interval_s", 0.0),
                    )
                elif operation == "submit_path":
                    result = connection.submit_path(
                        request["path_rad"],
                        request["speed_scale"],
                        request["max_velocity_rad_s"],
                        request["max_acceleration_rad_s2"],
                        request["tolerance_rad"],
                    )
                elif operation == "submit_trajectory":
                    result = connection.submit_trajectory(
                        request["times_s"],
                        request["positions_rad"],
                        request["command_rate_hz"],
                        request["max_step_rad"],
                        request["speed_scale"],
                        request["tolerance_rad"],
                    )
                elif operation == "effector_submit":
                    result = connection.effector_submit(request["positions"], request["duration_s"])
                elif operation == "effector_connect":
                    result = connection.effector_connect(request.get("feedback", "position"))
                elif operation in dispatch:
                    result = dispatch[operation]()
                else:
                    raise RuntimeError("unsupported device operation")
            if time.monotonic_ns() >= request["deadline_ns"]:
                raise RuntimeError("operation exceeded deadline; execution may have occurred")
            response.update(ok=True, data=result)
        except Exception as error:  # SDK failures become explicit protocol errors, never success.
            response["error"] = f"{type(error).__name__}: {error}"[:2000]
            # Nothing reached the device: the owner may keep this connection and its state.
            response["rejected"] = not connection.wrote
        writer.write((json.dumps(response, allow_nan=False, separators=(",", ":")) + "\n").encode())
        writer.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", required=True)
    parser.add_argument("--channel", required=True)
    args = parser.parse_args()
    if (
        not args.channel
        or len(args.channel) > 15
        or any(
            character not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for character in args.channel
        )
    ):
        parser.error("invalid channel")
    # Terminal Ctrl+C and service stop signals belong to the C++ owner, which must still be
    # able to send a hold through this process. The worker exits only on owner EOF.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    os.set_blocking(0, True)
    connection = AgxConnection(args.sdk_root, args.channel)
    try:
        serve(connection, sys.stdin.buffer, sys.stdout.buffer)
    finally:
        with contextlib.redirect_stdout(sys.stderr):
            connection.close()  # Closing transport is NOT proof that the controller stopped.


if __name__ == "__main__":
    main()
