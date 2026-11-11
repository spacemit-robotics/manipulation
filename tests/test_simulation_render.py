#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise viewer, real RGB-D transfer and device control in one simulated world."""

import json
import math
import os
from pathlib import Path
import select
import shutil
import struct
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def run(arguments, environment):
    return subprocess.run(arguments, env=environment, check=True, capture_output=True, text=True, timeout=20).stdout


def main():
    with tempfile.TemporaryDirectory(prefix="simulation-render-") as directory:
        directory = Path(directory)
        fixture = ROOT / "tests/fixtures/mujoco"
        scene = (
            (fixture / "scene.xml")
            .read_text()
            .replace(
                "<worldbody>",
                '<worldbody><camera name="overview" pos="0 0 3"/>'
                '<light pos="0 0 2"/><geom type="plane" size="2 2 .1" rgba=".1 .3 .5 1"/>',
            )
        )
        (directory / "scene.xml").write_text(scene)
        shutil.copyfile(fixture / "mapping.yaml", directory / "mapping.yaml")
        plan = {
            "schema_version": 3,
            "steps": [],
            "device": {
                "arm": {
                    "driver": "mujoco",
                    "device_id": "arm",
                    "resource_id": "render_test",
                    "joints": [
                        {"name": name, "minimum_rad": -1, "maximum_rad": 1} for name in ("second_joint", "first_joint")
                    ],
                    "timeout_s": 5,
                    "tolerance_rad": 0.005,
                    "options": {"mapping_file": "mapping.yaml"},
                }
            },
        }
        local = directory / "local.json"
        local.write_text(json.dumps(plan))
        environment = dict(os.environ, MANIPULATION_LOCK_DIRECTORY=str(directory / "locks"))
        server = subprocess.Popen(
            [os.environ["MANIPULATION_SIMULATION"], "--plan", str(local), "--port", "0", "--viewer"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            ready, _, _ = select.select([server.stdout], [], [], 15)
            if not ready:
                raise AssertionError("rendered server did not become ready")
            line = server.stdout.readline()
            if not line:
                raise AssertionError(server.stderr.read())
            port = json.loads(line)["port"]
            plan["device"]["arm"]["options"].update(transport="tcp", host="127.0.0.1", port=str(port))
            plan["steps"] = [{"capability": "joint_point", "target_rad": [0.1, -0.1], "speed_scale": 0.8}]
            remote = directory / "remote.json"
            remote.write_text(json.dumps(plan))
            previous_sample = 0
            for index in range(2):
                output = directory / f"frame-{index}"
                run(
                    [
                        os.environ["MANIPULATION_SIMULATION_CAMERA"],
                        "--port",
                        str(port),
                        "--width",
                        "160",
                        "--height",
                        "120",
                        "--output",
                        str(output),
                    ],
                    environment,
                )
                meta = json.loads((output / "metadata.json").read_text())["data"]
                assert (meta["width"], meta["height"]) == (160, 120)
                assert meta["sample_id"] > previous_sample
                previous_sample = meta["sample_id"]
                assert len(meta["K"]) == 9 and meta["K"][0] > 0
                assert len(meta["T_world_camera"]) == 16
                header = b"P6\n160 120\n255\n"
                rgb = (output / "rgb.ppm").read_bytes()
                assert rgb.startswith(header) and len(rgb) == len(header) + 160 * 120 * 3
                assert len(set(rgb[len(header) :])) > 10
                depth = struct.unpack("<19200f", (output / "depth.f32").read_bytes())
                assert all(math.isfinite(value) and value >= 0 for value in depth)
                assert any(1.8 < value < 3.1 for value in depth)
                run([os.environ["MANIPULATION_DEVICE"], "inspect", str(remote)], environment)
                if index == 0:
                    result = run([os.environ["MANIPULATION_DEVICE"], "run", str(remote)], environment)
                    assert json.loads(result.splitlines()[-1])["status"] == "succeeded"
            print("viewer, RGB-D pixels, advancing samples and remote device access passed")
        finally:
            server.terminate()
            try:
                server.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                server.kill()
                server.communicate(timeout=3)
                raise AssertionError("rendered server did not stop")


if __name__ == "__main__":
    main()
