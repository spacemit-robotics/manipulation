#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise installed operator commands with Mock devices and offline resource fixtures only."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) if not isinstance(value, str) else value)


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def pose(x, y, z, angle):
    vector = [x * angle, y * angle, z * angle]
    length = math.sqrt(sum(value * value for value in vector))
    u, v, w = [value / length for value in vector]
    c, s, d = math.cos(length), math.sin(length), 1 - math.cos(length)
    return [
        [c + u * u * d, u * v * d - w * s, u * w * d + v * s, x * 0.1],
        [v * u * d + w * s, c + v * v * d, v * w * d - u * s, y * 0.2],
        [w * u * d - v * s, w * v * d + u * s, c + w * w * d, z * 0.3],
        [0, 0, 0, 1],
    ]


def inverse(p):
    rotation = [[p[j][i] for j in range(3)] for i in range(3)]
    return [row + [-sum(row[k] * p[k][3] for k in range(3))] for row in rotation] + [[0, 0, 0, 1]]


def dataset():
    camera, target = pose(1, -1, 1, 0.3), pose(0, 0, 1, 0.2)
    samples = []
    for index in range(20):
        arm = pose(math.sin(index * 0.7), math.cos(index * 0.9), math.sin(index * 0.4), 0.8)
        observation = multiply(multiply(inverse(camera), arm), target)
        samples.append(dict(T_base_flange=sum(arm, []), T_camera_target=sum(observation, [])))
    return dict(schema_version=1, setup="eye_to_hand", samples=samples)


def check(prefix):
    prefix = Path(prefix).resolve()
    layout = json.loads((prefix / ".spacemit-manipulation.json").read_text())
    binary = prefix / layout["bindir"]
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    environment.pop("MANIPULATION_SITE", None)
    environment.pop("PYTHONPATH", None)
    environment.pop("LD_LIBRARY_PATH", None)
    with tempfile.TemporaryDirectory(prefix="operator commands ") as temporary:
        root = Path(temporary)
        site, source = root / "site", root / "algorithms"

        def command(name, *arguments, expected=0, stdin=None, error=None):
            result = subprocess.run(
                [str(binary / name), "--site", str(site), *map(str, arguments)],
                cwd=root,
                env=environment,
                input=stdin,
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != expected:
                raise AssertionError(str(arguments) + "\n" + result.stdout + result.stderr)
            if error is not None and error not in result.stderr:
                raise AssertionError(str(arguments) + "\n" + result.stderr)
            return result.stdout

        for name in ("manipulation", "hand-eye", "dice-arena"):
            assert command(name, "--help").startswith("usage: " + name + " ")
        for operation in (
            "check",
            "demo",
            "configure",
            "use",
            "validate",
            "inspect",
            "capabilities",
            "arm-check",
            "enable",
            "disable",
            "hand-enable",
            "hand-disable",
            "hold",
            "hand-hold",
            "joint-step",
            "joint-move",
            "hand-move",
            "hand-step",
            "hand-grasp",
            "pose",
            "monitor",
            "ik",
            "pose-move",
            "line-move",
            "teach",
            "fk",
            "manifest",
        ):
            assert command("manipulation", operation, "--help").startswith("usage: ")
        command("manipulation", "ik", 1, expected=2, error="the following arguments are required")
        command("manipulation", "check", "--profile", "build")
        assert '"status":"succeeded"' in command("manipulation", "demo")
        assert command("dice-arena", "mock").count('"status": "completed"') == 3
        command("manipulation", "configure", "unknown_model", expected=2, error="available: nero")
        command(
            "manipulation",
            "configure",
            "nero",
            "--variant",
            "missing",
            expected=2,
            error="available: arm, revo2_right",
        )
        command(
            "manipulation", "validate", expected=2, error="manipulation configure MODEL"
        )  # No device yet.
        command(
            "manipulation", "configure", "nero", "--variant", "revo2_right", "--set", "channel=can1"
        )
        assert (
            os.readlink(site / "device.yaml") == "nero/device.yaml"
        )  # Configuring selects the device.
        device = (site / "device.yaml").read_text()
        assert "channel: can1" in device and "agx_can1" in device, device
        # Resources are written relative to device.yaml and must resolve into this installation.
        for key, expected in (("sdk_root", "pyagxarm"), ("worker", "manipulation/agx_worker.py")):
            value = re.search(r"(?m)^\s*" + key + r":\s*(\S+)", device).group(1)
            resolved = (site / "nero" / value).resolve()
            assert not Path(value).is_absolute() and resolved.is_relative_to(prefix), (key, value)
            assert resolved.as_posix().endswith(expected) and resolved.exists(), (key, resolved)
        command("manipulation", "validate")  # Offline validation of Nero resources is safe.
        command(
            "manipulation", "configure", "nero", expected=2, error="manipulation use nero"
        )  # Never overwrites.
        # SO101 templates name the calibration files so101-calibrate keeps beside the SO101
        # configuration.
        command(
            "manipulation",
            "configure",
            "so101",
            "--variant",
            "gripper",
            "--set",
            "port=/dev/null",
            expected=2,
            error="so101-calibrate capture",
        )  # Not calibrated yet.
        assert not (site / "so101/device.yaml").exists()
        assert (
            os.readlink(site / "device.yaml") == "nero/device.yaml"
        )  # A failed configure selects nothing.
        command(
            "so101-calibrate",
            "limits",
            "--port",
            "/dev/null",
            expected=1,
            error=str(site / "so101"),
        )
        write(
            site / "so101/so101_calibration.json",
            {
                name: dict(id=index + 1, homing_offset=0, range_min=300, range_max=3800)
                for index, name in enumerate(
                    ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
                )
            },
        )
        write(
            site / "so101/so101_gripper_calibration.json",
            dict(motor_id=6, open_ticks=3400, closed_ticks=2000),
        )
        command(
            "manipulation", "configure", "so101", "--variant", "gripper", "--set", "port=/dev/null"
        )
        assert 'arm_calibration: "so101_calibration.json"' in (site / "device.yaml").read_text()
        # `use` lists the configured devices and switches among them; each keeps its own files.
        assert command("manipulation", "use") == "  nero\n* so101\n"
        assert command("manipulation", "use", "nero") == "* nero\n  so101\n"
        assert "agx_can1" in (site / "device.yaml").read_text()
        command("manipulation", "use", "missing", expected=2, error="configured: nero, so101")
        command("manipulation", "use", "..", expected=2, error="configured: nero, so101")
        assert os.readlink(site / "device.yaml") == "nero/device.yaml"
        (site / "device.yaml").unlink()
        command("manipulation", "validate", expected=2, error="manipulation use NAME")
        write(site / "device.yaml", "plain")  # Never replaces a file it did not create.
        command("manipulation", "use", "nero", expected=2, error="plain file")
        command("manipulation", "configure", "nero", expected=2, error="plain file")
        assert (site / "device.yaml").read_text() == "plain"
        (site / "device.yaml").unlink()

        # Every subsequent device operation uses an explicit seven-axis Mock, never the Nero file
        # above.
        plan = dict(
            schema_version=3,
            device=dict(
                driver="mock_device",
                model="nero",
                arm=dict(
                    device_id="arm",
                    resource_id="mock_operator",
                    joints=[
                        dict(name=f"joint{i}", minimum_rad=-3, maximum_rad=3) for i in range(1, 8)
                    ],
                ),
                effector=dict(
                    device_id="hand",
                    channels=[
                        "thumb_tip",
                        "thumb_base",
                        "index_finger",
                        "middle_finger",
                        "ring_finger",
                        "pinky_finger",
                    ],
                ),
            ),
            steps=[],
        )
        write(site / "mock/device.yaml", plan)
        command("manipulation", "use", "mock")
        command("manipulation", "inspect")
        assert "cancel=cancelled" in command(
            "manipulation", "arm-check"
        )  # Runs directly, no opt-in.
        command(
            "manipulation", "hand-step", "2", "0.01", "--allow-motion", expected=2
        )  # Removed opt-in.
        command("manipulation", "hand-step", "6", "0.01", expected=2)
        command("manipulation", "hand-step", "2", "nan", expected=2)
        for channel in range(6):  # Moves at once: no flag, no typed confirmation.
            output = command("manipulation", "hand-step", str(channel), "-0.01")
            assert "'status': 'succeeded'" in output
        output = command("manipulation", "hand-step", "pinky_finger", "-0.01")
        assert "'status': 'succeeded'" in output
        # A Mock hand starts open (1) on every connection, so an opening step is refused before
        # submission.
        command("manipulation", "hand-step", "pinky_finger", "0.01", expected=2)
        # Open-loop hand: relative steps are refused, absolute moves are delivered and reported
        # unverified.
        open_loop = json.loads(json.dumps(plan))
        open_loop["device"]["effector"]["feedback"] = "none"
        write(site / "open_loop.yaml", open_loop)
        command(
            "manipulation", "hand-step", "2", "0.01", "--plan", site / "open_loop.yaml", expected=2
        )
        command(
            "manipulation", "hand-move", *["0.2"] * 5, "--plan", site / "open_loop.yaml", expected=2
        )
        output = command(
            "manipulation",
            "hand-move",
            *["0.2"] * 6,
            "--plan",
            site / "open_loop.yaml",
            "--duration",
            "0.2",
        )
        assert "unknown (no feedback)" in output and "'status': 'unverified'" in output, output
        command("manipulation", "teach", "pose.json")
        command("manipulation", "teach", "pose.json", expected=2)
        command("manipulation", "fk", "pose.json")
        assert (
            site / "mock/pose.json"
        ).is_file()  # Poses stay with the device they were recorded on.
        # Cartesian tools on the Mock (all joints 0 at each connection): read-only pose and IK, then
        # moves.
        tip = json.loads(command("manipulation", "pose"))["position_m"]
        offset = [tip[0] + 0.02, tip[1], tip[2] - 0.01]
        assert json.loads(command("manipulation", "ik", *offset))["converged"]
        assert json.loads(command("manipulation", "ik", *offset, "--seed-pose", "pose.json"))[
            "converged"
        ]
        command("manipulation", "ik", 3, 0, 0, expected=2, error="no joint solution")
        command("manipulation", "pose-move", 3, 0, 0, expected=2, error="nothing moved")
        assert "'status': 'succeeded'" in command("manipulation", "pose-move", *offset)
        output = command("manipulation", "line-move", *offset, "--step-mm", 5)
        assert "5 waypoints" in output and "'status': 'succeeded'" in output, output
        command(
            "manipulation", "hand-grasp", expected=2, error="no grasp evidence"
        )  # The Mock hand has none.
        output = command("manipulation", "monitor", "--count", 3, "--rate", 20)
        assert output.count("joints_deg") == 3 and "pinky_finger 1.000" in output, output
        command("manipulation", "monitor", "--rate", 50, expected=2)

        values = dataset()
        write(site / "calibration_01/dataset.json", values)
        write(
            site / "calibration_01/provenance.json",
            dict(camera=dict(frame="color_optical", serial="fixture")),
        )
        command("hand-eye", "solve", "calibration_01")
        command("hand-eye", "solve", "calibration_01", expected=2)
        command("hand-eye", "promote", "calibration_01")
        command("hand-eye", "promote", "calibration_01", expected=2)
        command("hand-eye", "export", "calibration_01")
        command("hand-eye", "restore", "calibration_01")
        values["samples"][0]["T_camera_target"][3] += 0.03
        write(site / "bad/dataset.json", values)
        command("hand-eye", "solve", "bad", expected=1)
        command("hand-eye", "promote", "bad", expected=2)
        assert not (site / "bad/active.json").exists()

        green = dict(green_cup=dict(open_targets_0_100=[0] * 6, grip_targets_0_100=[10] * 6))
        write(source / "configs/green_cup.json", green)
        write(
            source / "configs/actions/gestures/result_feedback.json",
            dict(
                gestures={name: dict(hand_0_100=[0] * 6) for name in ("yeah", "thumbs-up", "tie")}
            ),
        )
        write(source / "cup_grasp_demo/__init__.py", "")
        write(source / "cup_grasp_demo/flow/__init__.py", "")
        write(
            source / "cup_grasp_demo/flow/green_pipeline.py",
            "import json\nfrom pathlib import Path\ndef load_config(path):\n"
            "    return json.loads(Path(path).read_text())\n"
            "def validate(config):\n    return config['green_cup']\n",
        )
        command("dice-arena", "cycle", expected=2)
        command(
            "dice-arena", "--source", source, "configure", expected=2
        )  # No site pipeline before `apply`.
        write(site / "green_cup.json", green)  # Normally created by `calibrate.sh apply`.
        command("manipulation", "use", "so101")  # The application is written for the Nero model.
        command(
            "dice-arena", "--source", source, "configure", expected=2, error="manipulation use nero"
        )
        command("manipulation", "use", "mock")
        command("dice-arena", "--source", source, "configure")
        command("dice-arena", "--source", source, "configure", expected=1)
        application = json.loads((site / "cup_application.json").read_text())
        assert application["device_plan"] == "mock/device.yaml", (
            application
        )  # Unaffected by a later `use`.
        assert application["planner"]["config"] == "green_cup.json", application["planner"]
        # Replace only the test application with the known installed Mock cycle to exercise the
        # short aliases.
        mock = prefix / layout["datadir"] / "dice_arena/config/mock.json"
        application = json.loads(mock.read_text())
        for key in ("executable", "device_plan", "policy_library"):
            application[key] = str((mock.parent / application[key]).resolve())
        write(site / "cup_application.json", application)
        command("dice-arena", "inspect")
        assert command("dice-arena", "cycle").count('"status": "completed"') == 3
        assert '"at_home": true' in command(
            "dice-arena", "home"
        )  # The original home action, no camera.

        # The game runs from the installation: no game source, staging or copy.
        command("dice-arena", "stage", expected=2)
        command("dice-arena", "serve", "--allow-motion", expected=2)  # Removed opt-in.
        game = prefix / layout["datadir"] / "dice_arena/game"
        if not (
            game / "vision/yolov8_objdetect/bin/yolov8_camera"
        ).is_file():  # Built on the K3 only.
            command("dice-arena", "serve", expected=2)
        assert '"mock_cycle": "passed"' in command("dice-arena", "check")
        assert not (site / "arena").exists()
        token = "a" * 32
        command(
            "dice-arena",
            "ack-grip",
            token,
            site / "wrong.json",
            "--operator",
            "fixture",
            expected=1,
        )
        command("dice-arena", "ack-grip", token, site / (token + ".json"), "--operator", "fixture")
    print(
        "operator commands: passed (Mock, quality gates, immutable outputs, permissions and "
        "offline app fixtures)"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix")
    check(parser.parse_args().prefix)


if __name__ == "__main__":
    main()
