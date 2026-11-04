"""One persistent C++ device owner, fixed operations and correlated bounded requests."""
import json
from pathlib import Path
import subprocess
import threading
import time


class DeviceCommandError(RuntimeError):
    """The structured control result, including stop_unknown, is retained for applications."""

    def __init__(self, result):
        self.result = result
        super().__init__(str(result))


class DeviceClient:
    """No startup motion. allow_motion=False opens the device read-only: every motion is refused.
    Enabling the arm is a separate permission, never inferred from a driver id.

    The device process holds the host-wide lease for the plan's resource_id until close().
    """

    def __init__(self, executable, plan, *, allow_motion=True, allow_enable=False,
                 allow_disable=False, allow_effector_enable=False, allow_effector_disable=False):
        if not allow_motion and any((allow_enable, allow_disable, allow_effector_enable, allow_effector_disable)):
            raise ValueError("torque permissions also require motion permission")
        self._condition = threading.Condition()
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._next_id = 0
        self._responses = {}
        self._pending = set()
        self._ignored = set()
        self._ready = False
        self.metadata = {}
        self._error = None
        self._closed = False
        command = [str(executable), "serve", str(Path(plan).resolve())]
        if not allow_motion:
            command.append("--read-only")
        if allow_enable:
            command.append("--enable")
        if allow_disable:
            command.append("--allow-disable")
        if allow_effector_enable:
            command.append("--allow-effector-enable")
        if allow_effector_disable:
            command.append("--allow-effector-disable")
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, encoding="utf-8", bufsize=1)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        with self._condition:
            ready = self._condition.wait_for(lambda: self._ready or self._error, timeout=8)
            error = self._error
        if not ready or error:
            self.close()
            raise RuntimeError(error or "C++ device runtime did not become ready")

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(65537)
                if not line:
                    raise RuntimeError("device process EOF; physical stop is not proven")
                if len(line) > 65536 or not line.endswith("\n"):
                    raise RuntimeError("invalid device response frame")
                response = json.loads(line)
                if response.get("v") != 1:
                    raise RuntimeError("device protocol version mismatch")
                with self._condition:
                    if response.get("event") == "ready":
                        if self._ready:
                            raise RuntimeError("duplicate device handshake")
                        self.metadata = response
                        self._ready = True
                    elif response.get("id") == 0:
                        self._error = str(response.get("error", "protocol rejected"))
                    else:
                        identifier = response.get("id")
                        if not self._ready or type(identifier) is not int or type(response.get("ok")) is not bool:
                            raise RuntimeError("invalid device response identity")
                        if identifier in self._ignored:
                            self._ignored.remove(identifier)
                        elif identifier in self._pending and identifier not in self._responses:
                            self._responses[identifier] = response
                        else:
                            raise RuntimeError("unexpected or duplicate device response")
                    self._condition.notify_all()
        except Exception as error:
            with self._condition:
                # EOF after a structured refusal must not erase the original startup/protocol error.
                self._error = self._error or str(error)
                self._condition.notify_all()

    def _send(self, operation, deadline, *, ignore_reply=False, **fields):
        with self._write_lock:
            self._next_id += 1
            identifier = self._next_id
            message = dict(v=1, id=identifier, op=operation, deadline_ns=deadline, **fields)
            encoded = json.dumps(message, allow_nan=False, separators=(",", ":")) + "\n"
            if len(encoded.encode()) > 1 << 20:
                raise ValueError("request exceeds 1 MiB")
            with self._condition:
                (self._ignored if ignore_reply else self._pending).add(identifier)
            self.process.stdin.write(encoded)
            self.process.stdin.flush()
            return identifier

    def call(self, operation, *, timeout=10, is_cancelled=None, **fields):
        """Queue wait is included in timeout; cancellation never means a verified physical stop."""
        if timeout <= 0 or timeout > 180:
            raise ValueError("timeout must be in (0,180] seconds")
        deadline = time.monotonic() + timeout
        if not self._command_lock.acquire(timeout=timeout):
            raise TimeoutError("request expired waiting for device owner")
        try:
            if self._closed or self._error:
                raise RuntimeError(self._error or "device client closed")
            if time.monotonic() >= deadline or (is_cancelled and is_cancelled()):
                raise DeviceCommandError({"status": "cancelled", "motion_attempted": False})
            identifier = self._send(operation, int(deadline * 1e9), **fields)
            cancel_sent = False
            while True:
                with self._condition:
                    response = self._responses.pop(identifier, None)
                    if response is not None:
                        self._pending.discard(identifier)
                        if not response["ok"]:
                            raise DeviceCommandError(response["error"])
                        return response["data"]
                    if self._error:
                        raise DeviceCommandError({"status": "stop_unknown", "reason": self._error})
                    self._condition.wait(timeout=0.02)
                callback_error = None
                try:
                    cancelled = is_cancelled and is_cancelled()
                except Exception as error:
                    cancelled, callback_error = True, error
                if not cancel_sent and (time.monotonic() >= deadline or cancelled):
                    self._send("cancel", int((time.monotonic() + 4) * 1e9),
                               ignore_reply=True, target_id=identifier)
                    cancel_sent = True
                    if callback_error:
                        self._error = "cancellation callback failed; cancellation requested, stop unknown"
                        raise DeviceCommandError({"status": "stop_unknown", "reason": self._error}) from callback_error
                if time.monotonic() > deadline + 4:
                    self._error = "no bounded cancellation result; physical stop unknown"
                    raise DeviceCommandError({"status": "stop_unknown", "reason": self._error})
        finally:
            self._command_lock.release()

    def state(self, **kwargs):
        return self.call("state", **kwargs)

    def diagnose(self, **kwargs):
        return self.call("diagnose", **kwargs)

    def capabilities(self, **kwargs):
        """Return driver capabilities; unsupported operations are never emulated."""
        return self.call("capabilities", **kwargs)

    def enable_arm(self, **kwargs):
        """Explicitly enable the arm; requires allow_enable at construction."""
        return self._motion("enable", **kwargs)

    def disable_arm(self, **kwargs):
        """Release arm torque; requires allow_disable and driver support."""
        return self._motion("disable", **kwargs)

    def enable_effector(self, **kwargs):
        """Explicitly enable the tool; requires allow_effector_enable."""
        return self._motion("effector_enable", **kwargs)

    def disable_effector(self, **kwargs):
        """Release tool torque; requires allow_effector_disable."""
        return self._motion("effector_disable", **kwargs)

    def hold_arm(self, **kwargs):
        """Request a feedback-confirmed hold without releasing torque or clearing faults."""
        return self._motion("arm_hold", **kwargs)

    def hold_effector(self, **kwargs):
        """Request a feedback-confirmed tool hold; open-loop tools reject this operation."""
        return self._motion("effector_hold", **kwargs)

    def jog_arm(self, joint_index, delta_rad, speed_scale=0.05, **kwargs):
        """Step one zero-based joint relative to fresh feedback, applying normal joint limits."""
        if type(joint_index) is not int or joint_index < 0:
            raise ValueError("joint_index must be a nonnegative integer")
        return self._motion("arm_jog", joint_index=joint_index, delta_rad=delta_rad,
                            speed_scale=speed_scale, **kwargs)

    def move_arm(self, target_rad, speed_scale, *, tolerance_rad=0.0, wait_for_arrival=True, resend_interval_s=0.0,
                 **kwargs):
        """Point move; tolerance_rad (<= 0.8 deg; 0: the device tolerance) is the arrival tolerance.

        With wait_for_arrival=False the move completes on two fresh samples within tolerance_rad
        (detail "within_tolerance"), before the arm is still. resend_interval_s (0, or 0.02..0.2)
        re-sends the same target at that interval while beyond tolerance_rad.
        """
        return self._motion("arm_point", target_rad=list(target_rad), speed_scale=speed_scale,
                            tolerance_rad=tolerance_rad, wait_for_arrival=bool(wait_for_arrival),
                            resend_interval_s=float(resend_interval_s), **kwargs)

    def move_arm_trajectory(self, times_s, positions_rad, *, command_rate_hz=100.0, max_step_rad=0.0174533,
                            speed_scale=1.0, tolerance_rad=0.0, wait_for_arrival=True, effector_schedule=(),
                            allow_unverified=False, **kwargs):
        """Stream a timed joint trajectory (knots interpolated linearly at command_rate_hz).

        The first knot must be the current position (0.5 deg); no commanded step may exceed
        max_step_rad (<= 3 deg). Arrival at the last knot within tolerance_rad, as for paths; with
        wait_for_arrival=False the command completes once the last knot is sent (status succeeded,
        detail "delivered") and the next command may start while the arm settles.

        effector_schedule: (at_s, positions, duration_s) targets sent while the arm moves, an
        arm-and-hand gesture; the result is then "unverified", accepted only with allow_unverified=True.
        """
        schedule = [dict(at_s=float(at), positions=[float(v) for v in positions], duration_s=float(duration))
                    for at, positions, duration in effector_schedule]
        return self._motion("arm_trajectory", {"succeeded", "unverified"} if allow_unverified else {"succeeded"},
                            times_s=[float(t) for t in times_s],
                            positions_rad=[[round(float(v), 7) for v in q] for q in positions_rad],
                            command_rate_hz=command_rate_hz, max_step_rad=max_step_rad, speed_scale=speed_scale,
                            tolerance_rad=tolerance_rad, wait_for_arrival=bool(wait_for_arrival),
                            **({"effector_schedule": schedule} if schedule else {}), **kwargs)

    def move_arm_path(self, path_rad, speed_scale, max_velocity_rad_s, max_acceleration_rad_s2, *,
                      tolerance_rad=0.0, **kwargs):
        """Continuous motion through the waypoints, stopping only at the last one.

        The smooth joint-space profile is bounded by max_velocity_rad_s (<= 0.35) and
        max_acceleration_rad_s2 (<= 1.0) on the largest joint change; speed_scale is the controller ratio.
        tolerance_rad (<= 0.8 deg; 0: the device tolerance) is the arrival tolerance at the last waypoint.
        """
        return self._motion("arm_path", path_rad=[list(map(float, q)) for q in path_rad], speed_scale=speed_scale,
                            max_velocity_rad_s=max_velocity_rad_s, max_acceleration_rad_s2=max_acceleration_rad_s2,
                            tolerance_rad=tolerance_rad, **kwargs)

    def move_effector(self, positions, duration_s=0.5, *, allow_unverified=False, **kwargs):
        """An effector without feedback returns status "unverified" (delivered, not measured).

        Such a result is accepted only with allow_unverified=True; it never means the target was reached.
        """
        return self._motion("effector_position", {"succeeded", "unverified"} if allow_unverified else {"succeeded"},
                            positions=list(positions), duration_s=duration_s, **kwargs)

    def _motion(self, operation, accepted=frozenset({"succeeded"}), **kwargs):
        result = self.call(operation, **kwargs)
        if result.get("status") not in accepted:
            raise DeviceCommandError(result)
        return result

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.process.poll() is None:
            try:
                self._send("quit", time.monotonic_ns() + 4_000_000_000, ignore_reply=True)
                self.process.stdin.close()
            except (OSError, ValueError):
                pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("runtime did not exit; physical stop unknown, do not restart") from error
        self._reader.join(timeout=1)
        for stream in (self.process.stdin, self.process.stdout):
            if stream is not None and not stream.closed:
                try:
                    stream.close()
                except BrokenPipeError:
                    pass  # A failed child may exit before buffered quit bytes are flushed.

    def __enter__(self):
        return self

    def __exit__(self, exc_type, _value, _traceback):
        try:
            self.close()
        except RuntimeError:
            if exc_type is None:
                raise
