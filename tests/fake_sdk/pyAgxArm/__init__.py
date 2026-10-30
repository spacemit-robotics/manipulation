# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""TEST FIXTURE ONLY. No CAN, no production fallback. Exercises the real worker adapter."""

import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace as Node


class ArmModel:
    NERO = "nero"


class NeroFW:
    V120 = "v120"


def create_agx_arm_config(**kwargs):
    return kwargs


class FakeRobot:
    OPTIONS = Node(EFFECTOR=Node(REVO2="revo2"))

    def __init__(self, config):
        self.channel = config["channel"]
        self.mode = self.channel.split("_")[-1]
        self.position = [0.0] * 7
        self.automatic_mode, self.limits_enabled = True, False
        self._parser = Node()
        self.updated = time.monotonic()
        self.update()

    def update(self):
        mode = self.mode
        now = time.monotonic()
        if mode == "lag" and getattr(self, "commanded", None) is not None:
            # A real servo trails its targets (50 ms time constant) and keeps converging after the last one.
            gain = 1 - math.exp(-(now - self.updated) / 0.05)
            self.position = [p + gain * (c - p) for p, c in zip(self.position, self.commanded)]
        self.updated = now
        if mode == "recover":
            control = json.loads((Path(__file__).parent / "recovery.json").read_text(encoding="utf-8"))
            mode = control["mode"]
            if mode == "worker_exit":
                os._exit(23)
            if mode == "rpc_timeout":
                time.sleep(10)
            if mode == "truncated":
                os.write(1, b'{"v":')
                os._exit(24)
            if "joints" in control:
                self.position = list(control["joints"])
        stamp = time.time() - (10 if mode == "stale" else 0)
        for name, indices in (("joint_12", (1, 2)), ("joint_34", (3, 4)), ("joint_56", (5, 6)), ("joint_7", (7,))):
            setattr(self._parser, name, Node(timestamp=stamp, msg=Node(
                **{f"joint_{i}": self.position[i - 1] for i in indices})))
        if mode == "part":
            self._parser.joint_34 = None
        self.status = Node(timestamp=stamp, msg=Node(
            arm_status=0, ctrl_mode=1, motion_status=int(getattr(self, "not_reached", False) and self.position[0] == 0),
            err_code=int(mode == "fault"),
            mode_feedback=6 if mode == "mode" else 1))
        self.motors = [Node(timestamp=stamp, msg=Node(
            foc_status_code=(0 if mode == "disabled" else 64) | int(mode == "motor"),
            foc_status=Node(driver_enable_status=mode != "disabled"))) for _ in range(7)]

    def record(self, operation, **values):
        if self.mode in ("recover", "stall", "hnofb", "drop", "path", "offset", "nreach", "lag"):
            with (Path(__file__).parent / "recovery-events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"op": operation, **values}) + "\n")

    def connect(self):
        self.record("connect")

    def disconnect(self):
        self.record("disconnect")

    def get_arm_status(self):
        return self.status

    def get_driver_states(self, index):
        return self.motors[index - 1]

    def get_firmware(self, **_kwargs):
        return {"software_version": "1.20"}

    def get_joint_angle_vel_limits(self, joint, **_kwargs):  # Values as a K3 Nero reports them.
        return Node(timestamp=time.time(), msg=Node(min_angle_limit=-2.7, max_angle_limit=2.7,
                                                    max_joint_spd=3.14 if joint <= 4 else 3.92))

    def get_joint_acc_limits(self, joint, **_kwargs):
        return Node(timestamp=time.time(), msg=Node(max_joint_acc=5.0))

    def set_speed_percent(self, percent):
        self.record("set_speed_percent", percent=percent)
        if not 1 <= percent <= 100:
            raise ValueError("bad speed")

    def move_j(self, target):
        self.record("move_j", target=list(target))
        if self.mode == "write":
            raise RuntimeError("injected SDK write failure")
        if self.mode == "stall" and list(target) != self.position:
            return  # Far target accepted by the controller but never reached; holds still settle.
        if self.mode in ("drop", "nreach") and list(target) != self.position and not getattr(self, "dropped", False):
            self.dropped = True
            if self.mode == "nreach":
                self.not_reached = True  # Stands still short of the target, reporting "not reached".
                self.update()
            return  # The first frames of a new target do not take effect.
        self.position = list(target)
        self.update()

    def move_js(self, target):
        self.record("move_js", target=[round(v, 9) for v in target])
        if self.mode == "lag":
            self.commanded = list(target)
            return
        self.position = list(target)
        if self.mode == "offset":  # MIT-mode servo: a small steady-state error remains on joint 5.
            self.position[4] += 0.003
        self.update()

    def set_motion_mode(self, mode):
        self.record("set_motion_mode", mode=mode)

    def get_auto_set_motion_mode_enabled(self):
        return self.automatic_mode

    def set_auto_set_motion_mode_enabled(self, enabled):
        self.record("auto_motion_mode", enabled=enabled)
        self.automatic_mode = enabled

    def set_joint_limits_enabled(self, enabled):
        self.limits_enabled = enabled

    def get_joint_angles(self):
        return Node(timestamp=time.time(), msg=list(self.position))

    def enable(self, **_kwargs):
        self.record("enable")
        return True

    def init_effector(self, _kind):
        self.record("init_effector")
        return FakeHand(self)


class FakeHand:
    names = ("thumb_tip", "thumb_base", "index_finger", "middle_finger", "ring_finger", "pinky_finger")

    def __init__(self, arm):
        self.arm = arm
        self.positions = dict.fromkeys(self.names, 0)

    def get_finger_pos(self):
        if self.arm.mode in ("hmissing", "hnofb"):
            return None
        return Node(timestamp=time.time() - (10 if self.arm.mode == "hstale" else 0),
                    msg=Node(**self.positions))

    def get_finger_current(self):
        if self.arm.mode == "hdead":
            return None
        return Node(timestamp=time.time(), msg=Node(**dict.fromkeys(self.names, 0)))

    def get_hand_status(self):
        if self.arm.mode == "hnofb":
            return None
        return Node(timestamp=time.time(), msg=Node(left_or_right=1 if self.arm.mode == "hleft" else 2,
                    **dict.fromkeys(self.names, 2 if self.arm.mode == "hstall" else 0)))

    def position_time_ctrl(self, mode, **values):
        self.arm.record("hand_" + mode)
        if mode == "pos" and self.arm.mode != "hnoend":
            self.positions = {name: round(value * 255 / 100) for name, value in values.items()}  # 0..255


class AgxArmFactory:
    @staticmethod
    def create_arm(config):
        robot = FakeRobot(config)
        # Refresh frames lazily when parser is accessed (SDK receiver equivalent).
        class ReceivingRobot(FakeRobot):
            def __getattribute__(self, name):
                if name == "_parser" and not object.__getattribute__(self, "__dict__").get("_refreshing", False):
                    object.__setattr__(self, "_refreshing", True)
                    self.update()
                    object.__setattr__(self, "_refreshing", False)
                return super().__getattribute__(name)
        robot.__class__ = ReceivingRobot
        return robot
