#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the installed UR5e simulation after moving the installation."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


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


def check(prefix):
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
    print("simulation installation: UR5e, Robotiq, relocation and C++ consumer passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    arguments = parser.parse_args()
    check(arguments.prefix.resolve())


if __name__ == "__main__":
    main()
