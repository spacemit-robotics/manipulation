#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Check build/runtime dependencies without opening CAN, cameras or audio devices."""

import argparse
import importlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def check(profile, prefix=None):
    """Return named failures; importing a module does not validate attached hardware."""
    failures = []
    if sys.version_info < (3, 10):
        failures.append("Python 3.10 or newer is required")
    if profile == "build":
        for command in ("cmake", "c++", "pkg-config"):
            if shutil.which(command) is None:
                failures.append("missing command: " + command)
        if shutil.which("pkg-config"):
            result = subprocess.run(["pkg-config", "--exists", "yaml-cpp"], check=False)
            if result.returncode:
                failures.append("missing development package: yaml-cpp")
        if shutil.which("cmake"):
            output = subprocess.run(
                ["cmake", "--version"], check=True, capture_output=True, text=True
            ).stdout
            version = tuple(int(item) for item in output.splitlines()[0].split()[-1].split(".")[:2])
            if version < (3, 16):
                failures.append("CMake 3.16 or newer is required")
    else:
        modules = ["can", "typing_extensions", "numpy"]
        if profile == "vision":
            modules += ["scipy", "cv2", "pyrealsense2"]
            if (
                shutil.which("pkg-config")
                and subprocess.run(["pkg-config", "--exists", "opencv4"]).returncode
            ):
                failures.append(
                    "missing development package: opencv4 (needed to build calibration)"
                )
        for module in modules:
            try:
                loaded = importlib.import_module(module)
                if module == "cv2" and not hasattr(loaded, "aruco"):
                    failures.append("OpenCV ArUco support is required")
            except Exception as error:
                failures.append(module + ": " + str(error))
        if prefix is not None:
            layout = json.loads((prefix / ".spacemit-manipulation.json").read_text())
            aliases = {"amd64": "x86_64", "arm64": "aarch64"}
            expected_arch = layout.get("architecture", "").lower()
            actual_arch = platform.machine().lower()
            if aliases.get(expected_arch, expected_arch) != aliases.get(actual_arch, actual_arch):
                failures.append("installation architecture does not match this CPU")
            # The pinned vendor SDK is required only when the Nero driver was installed.
            if (prefix / layout["libexecdir"] / "manipulation/agx_worker.py").is_file():
                sdk = prefix / layout["datadir"] / "pyagxarm"
                marker = sdk / "UPSTREAM_COMMIT"
                expected = "e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b"
                if not marker.is_file() or marker.read_text().strip() != expected:
                    failures.append("fixed pyAgxArm package is missing or has a different revision")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("build", "control", "vision"), default="build")
    parser.add_argument("--prefix", type=Path)
    args = parser.parse_args()
    try:
        failures = check(args.profile, args.prefix)
    except (OSError, ValueError, KeyError) as error:
        failures = [str(error)]
    for failure in failures:
        print("MISSING: " + failure, file=sys.stderr)
    if failures:
        print(
            "Building with mm checks and installs the apt dependencies declared in package.xml.",
            file=sys.stderr,
        )
        if args.profile == "vision":
            print(
                "pyrealsense2 is not an apt package: install it for this Python and CPU.",
                file=sys.stderr,
            )
        return 1
    print("environment " + args.profile + ": passed (no hardware accessed)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
