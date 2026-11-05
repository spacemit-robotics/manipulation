#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Verify a moved installation from an unrelated cwd, using Mock devices only."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def run(arguments, directory):
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("LD_LIBRARY_PATH", None)
    environment.pop("MANIPULATION_PYTHON", None)
    result = subprocess.run(
        [str(value) for value in arguments],
        cwd=directory,
        env=environment,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=90,
    )
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def relative_paths(config, base, prefix):
    for key in ("executable", "device_plan", "policy_library", "grip_ack_directory"):
        if key in config:
            if Path(config[key]).is_absolute():
                raise AssertionError("non-relocatable application field: " + key)
            if not (base / config[key]).resolve().is_relative_to(prefix):
                raise AssertionError("resource left the relocated bundle: " + key)


def check(prefix):
    with tempfile.TemporaryDirectory(prefix="manipulation-relocation-") as temporary:
        temporary = Path(temporary)
        original = temporary / "original bundle"
        shutil.copytree(prefix, original, symlinks=True)
        moved = temporary / "relocated bundle"
        original.rename(moved)
        directory = temporary / "unrelated cwd"
        directory.mkdir()
        layout = json.loads((moved / ".spacemit-manipulation.json").read_text())
        assert not (moved / layout["bindir"] / "cup-shake").exists()
        assert not (moved / layout["datadir"] / "cup_shake").exists()
        for name in ("manipulation", "hand-eye", "dice-arena"):
            run([moved / layout["bindir"] / name, "--help"], directory)
        output = run([moved / layout["bindir"] / "manipulation", "demo"], directory)
        assert '"status":"succeeded"' in output, output
        output = run([moved / layout["bindir"] / "dice-arena", "mock"], directory)
        records = [json.loads(line) for line in output.splitlines() if line.startswith("{")]
        assert sum(record.get("status") == "completed" for record in records) == 3, output
        assert not any(record.get("physical_grip_verified") for record in records)
        app = moved / layout["datadir"] / "dice_arena"
        for script in ("calibrate.sh", "calibrate_web.sh", "calibration_web_service.sh"):
            output = run(["bash", app / script, "--help"], directory)
            assert "usage:" in output.lower(), output
        output = run(
            [
                moved / layout["bindir"] / "manipulation-device",
                "run",
                moved
                / layout["datadir"]
                / "manipulation/examples/point_sequence/point_sequence.yaml",
            ],
            directory,
        )
        assert '"status":"succeeded"' in output, output
        config_file = moved / layout["datadir"] / "dice_arena/config/mock.json"
        relative_paths(json.loads(config_file.read_text()), config_file.parent, moved)
        for variant in ("arm", "revo2_right"):
            site = moved / "site" / variant
            run(
                [
                    moved / layout["bindir"] / "manipulation",
                    "--site",
                    site,
                    "configure",
                    "nero",
                    "--variant",
                    variant,
                ],
                directory,
            )
            content = (site / "device.yaml").read_text()
            assert str(prefix) not in content and str(moved) not in content

        # Commands work through a symlinked prefix and a symlink to a command.
        linked = temporary / "linked prefix"
        linked.symlink_to(moved, target_is_directory=True)
        run([linked / layout["bindir"] / "manipulation", "demo"], directory)
        shortcut = temporary / "shortcut"
        shortcut.mkdir()
        (shortcut / "manipulation").symlink_to(moved / layout["bindir"] / "manipulation")
        run([shortcut / "manipulation", "demo"], directory)

        # The installed public-interface example must not need a source checkout.
        consumer = moved / layout["datadir"] / "manipulation/examples/device_consumer"
        build = temporary / "consumer build"
        run(
            [
                "cmake",
                "-S",
                consumer,
                "-B",
                build,
                "-DCMAKE_PREFIX_PATH=" + str(moved),
                "-Dmanipulation_DIR=" + str(moved / layout["libdir"] / "cmake/manipulation"),
            ],
            directory,
        )
        run(["cmake", "--build", build, "-j2"], directory)
        output = run(
            [
                build / "device-state",
                moved / layout["datadir"] / "manipulation/examples/mock_device.yaml",
            ],
            directory,
        )
        assert "joints=2 moving=0" in output, output

        # Exercise the installed site generator with an offline algorithm fixture.
        source = moved / "assets/algorithms"
        pipeline = source / "cup_grasp_demo/flow"
        pipeline.mkdir(parents=True)
        (source / "cup_grasp_demo/__init__.py").write_text("")
        (pipeline / "__init__.py").write_text("")
        (pipeline / "green_pipeline.py").write_text(
            "import json\nfrom pathlib import Path\ndef load_config(path):\n"
            "    return json.loads(Path(path).read_text())\n"
            "def validate(config):\n    return config\n"
        )
        green = source / "green.json"
        green.write_text(json.dumps(dict(open_targets_0_100=[0] * 6, grip_targets_0_100=[10] * 6)))
        gestures = source / "configs/actions/gestures"
        gestures.mkdir(parents=True)
        (gestures / "result_feedback.json").write_text(
            json.dumps(
                dict(
                    gestures={
                        name: dict(hand_0_100=[0] * 6) for name in ("yeah", "thumbs-up", "tie")
                    }
                )
            )
        )
        application = moved / "site/application.json"
        generator = moved / layout["datadir"] / "dice_arena/tools/configure_site.py"
        plan = moved / layout["datadir"] / "manipulation/examples/mock_device.yaml"
        run(
            ["python3", generator, moved, plan, source, green, moved / "site/runtime", application],
            directory,
        )
        site = json.loads(application.read_text())
        assert all(
            not Path(site[key]).is_absolute()
            for key in ("executable", "device_plan", "policy_library", "grip_ack_directory")
        )
        assert all(
            not Path(site["planner"][key]).is_absolute()
            for key in ("source_root", "config", "session_directory")
        )

        # The installed game runs from the moved prefix with the Mock cup application.
        output = run(
            [moved / layout["bindir"] / "dice-arena", "--site", moved / "site", "check"], directory
        )
        assert '"mock_cycle": "passed"' in output, output

    print(
        "relocation: passed (Mock, public C++ consumer, relative device/application/game "
        "configurations)"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    args = parser.parse_args()
    check(args.prefix.resolve())


if __name__ == "__main__":
    main()
