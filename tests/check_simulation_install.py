#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the installed UR5e simulation after moving the installation."""

import argparse
import ctypes
import ctypes.util
import json
import math
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

import yaml


def run(arguments, directory, environment):
    result = subprocess.run(
        [str(argument) for argument in arguments],
        cwd=directory,
        env=environment,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def window_pixels():
    """Sample the private Xvfb framebuffer, including the visible viewer."""
    x11 = ctypes.CDLL(ctypes.util.find_library("X11"))
    functions = {
        "XOpenDisplay": ([ctypes.c_char_p], ctypes.c_void_p),
        "XDefaultRootWindow": ([ctypes.c_void_p], ctypes.c_ulong),
        "XDefaultScreen": ([ctypes.c_void_p], ctypes.c_int),
        "XDisplayWidth": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "XDisplayHeight": ([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        "XGetImage": ([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                       ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int], ctypes.c_void_p),
        "XGetPixel": ([ctypes.c_void_p, ctypes.c_int, ctypes.c_int], ctypes.c_ulong),
        "XDestroyImage": ([ctypes.c_void_p], ctypes.c_int),
        "XCloseDisplay": ([ctypes.c_void_p], ctypes.c_int),
    }
    for name, (arguments, result) in functions.items():
        function = getattr(x11, name)
        function.argtypes = arguments
        function.restype = result
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("cannot open the private Xvfb display")
    try:
        screen = x11.XDefaultScreen(display)
        width = x11.XDisplayWidth(display, screen)
        height = x11.XDisplayHeight(display, screen)
        image = x11.XGetImage(display, x11.XDefaultRootWindow(display), 0, 0,
                             width, height, ctypes.c_ulong(-1), 2)
        if not image:
            raise RuntimeError("cannot read the viewer framebuffer")
        try:
            return tuple(x11.XGetPixel(image, x, y)
                         for y in range(0, height, max(1, height // 120))
                         for x in range(0, width, max(1, width // 160)))
        finally:
            x11.XDestroyImage(image)
    finally:
        x11.XCloseDisplay(display)


def wait_ready(server, log):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        for line in log.read_text().splitlines():
            if line.startswith("{"):
                record = json.loads(line)
                if record.get("event") == "ready":
                    return record["port"]
        if server.poll() is not None:
            break
        time.sleep(0.05)
    raise AssertionError("viewer did not become ready:\n" + log.read_text())


def capture(camera, port, output, environment):
    run([camera, "--port", port, "--camera", "overview", "--width", "160", "--height", "120",
         "--output", output], output.parent, environment)
    metadata = json.loads((output / "metadata.json").read_text())["data"]
    assert (metadata["width"], metadata["height"]) == (160, 120)
    header = b"P6\n160 120\n255\n"
    rgb = (output / "rgb.ppm").read_bytes()
    assert rgb.startswith(header) and len(rgb) == len(header) + 160 * 120 * 3
    assert len(set(rgb[len(header):])) > 10, "UR5e RGB image is empty"
    depth = struct.unpack("<19200f", (output / "depth.f32").read_bytes())
    assert all(math.isfinite(value) and value >= 0 for value in depth)
    assert sum(0.1 < value < 5.0 for value in depth) > 100, "UR5e depth image is empty"
    return metadata


def check_render(bindir, data, directory, environment):
    environment = {name: value for name, value in environment.items() if not name.startswith("LIBGL_")}
    log = directory / "viewer.log"
    plan = data / "examples/simulation/ur5e_robotiq.yaml"
    with log.open("w") as output:
        server = subprocess.Popen(
            [str(bindir / "manipulation-simulation"), "--plan", str(plan), "--port", "0", "--viewer"],
            cwd=directory, env=environment, stdin=subprocess.DEVNULL,
            stdout=output, stderr=subprocess.STDOUT,
        )
        try:
            port = wait_ready(server, log)
            camera = bindir / "manipulation-simulation-camera"
            first = capture(camera, port, directory / "frame-before", environment)
            before = window_pixels()
            assert len(set(before)) > 10, "UR5e viewer framebuffer is empty"
            remote = yaml.safe_load(plan.read_text())
            for role in remote["device"].values():
                mapping = (plan.parent / role["options"]["mapping_file"]).resolve()
                role["options"].update(mapping_file=str(mapping), transport="tcp", host="127.0.0.1", port=str(port))
            remote["steps"] = remote["steps"][:1]
            remote["steps"][0]["target_rad"][0] = -1.0
            remote_plan = directory / "remote.json"
            remote_plan.write_text(json.dumps(remote))
            motion = run([bindir / "manipulation", "run", "--plan", remote_plan], directory, environment)
            assert json.loads(motion.splitlines()[-1])["status"] == "succeeded", motion
            second = capture(camera, port, directory / "frame-after", environment)
            assert second["sample_id"] > first["sample_id"], "rendered world did not advance"
            deadline = time.monotonic() + 10
            changed = 0
            while time.monotonic() < deadline and server.poll() is None:
                changed = sum(left != right for left, right in zip(before, window_pixels()))
                if changed > 20:
                    break
                time.sleep(0.05)
            assert changed > 20, "viewer did not redraw the moving UR5e"
            assert server.poll() is None, log.read_text()
        except Exception:
            print(log.read_text(), file=sys.stderr)
            raise
        finally:
            server.terminate()
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)
                raise AssertionError("viewer did not stop on SIGTERM") from None
        assert server.returncode == 0, log.read_text()
        assert "OpenGL error" not in log.read_text(), log.read_text()
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", port))
        listener.listen(1)
    print("installed UR5e viewer: nonempty first frame, motion redraw, RGB-D and clean shutdown passed")


def check(prefix, render=False):
    with tempfile.TemporaryDirectory(prefix="simulation-relocation-") as temporary:
        temporary = Path(temporary)
        original = temporary / "original"
        shutil.copytree(prefix, original, symlinks=True)
        moved = temporary / "moved install"
        original.rename(moved)
        unrelated = temporary / "unrelated"
        unrelated.mkdir()
        environment = dict(os.environ)
        for name in ("LD_LIBRARY_PATH", "PYTHONPATH", "MANIPULATION_PYTHON", "MUJOCO_DIR"):
            environment.pop(name, None)
        environment["MANIPULATION_LOCK_DIRECTORY"] = str(temporary / "locks")
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        layout = json.loads((moved / ".spacemit-manipulation.json").read_text())
        executable = moved / layout["bindir"] / "manipulation"
        data = moved / layout["datadir"] / "manipulation"
        run(
            ["python3", data / "tools/prepare_simulation_assets.py", "--check"],
            unrelated,
            environment,
        )
        for name in ("ur5e_robotiq", "ur5e_arm"):
            plan = data / "examples/simulation" / (name + ".yaml")
            run([executable, "validate", "--plan", plan], unrelated, environment)
            output = run([executable, "run", "--plan", plan], unrelated, environment)
            records = [json.loads(line) for line in output.splitlines() if line.startswith("{")]
            if not records or records[-1].get("status") != "succeeded":
                raise AssertionError(output)
        # Verify public CMake exports without the source checkout or its original library paths.
        consumer = data / "examples/device_consumer"
        build = temporary / "consumer"
        run(
            ["cmake", "-S", consumer, "-B", build, "-DCMAKE_PREFIX_PATH=" + str(moved)],
            unrelated,
            environment,
        )
        run(["cmake", "--build", build, "-j2"], unrelated, environment)
        output = run(
            [build / "device-state", data / "examples/simulation/ur5e_arm.yaml"],
            unrelated,
            environment,
        )
        if "joints=6" not in output:
            raise AssertionError(output)
        if render:
            check_render(moved / layout["bindir"], data, unrelated, environment)
    print("simulation installation: UR5e, Robotiq, relocation and C++ consumer passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--render", action="store_true", help="also check the UR5e viewer and RGB-D in a private Xvfb")
    parser.add_argument("--xvfb-child", action="store_true", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.render and not arguments.xvfb_child:
        subprocess.run(
            ["xvfb-run", "-a", "-s", "-screen 0 1024x768x24 -nolisten tcp", sys.executable,
             str(Path(__file__).resolve()), str(arguments.prefix.resolve()), "--render", "--xvfb-child"],
            check=True, timeout=300,
        )
    else:
        check(arguments.prefix.resolve(), arguments.render)


if __name__ == "__main__":
    main()
