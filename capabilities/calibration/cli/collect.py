# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Stationary eye-to-hand sample collection. The arm is never moved, enabled or reconfigured.

The camera side is a sampler module selected with --sampler and must provide:
  create_sampler(options: dict[str, str]) -> sampler
  sampler.capture(port, prefix: Path) -> dict   # record with 4x4 T_base_flange and T_camera_board;
                                                # may write images named prefix + suffix
  sampler.provenance() -> dict                  # must include camera.frame and camera.serial
  sampler.dependencies() -> list[Path]          # files fingerprinted into provenance.json
  sampler.close()
`port` reads distinct fresh stationary arm feedback: read_joints() and read() (4x4 base->flange).
"""

import argparse
import hashlib
import importlib
from pathlib import Path
from spacemit_manipulation import DeviceClient
from spacemit_manipulation.sampling import FeedbackPort
from spacemit_manipulation.urdf_fk import UrdfKinematics
from spacemit_manipulation.workbench import load, save_new


def options(items):
    result = {}
    for item in items:
        name, separator, value = item.partition("=")
        if not separator or not name:
            raise ValueError(f"expected NAME=VALUE, got {item!r}")
        result[name] = value
    return result


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    for name in ("executable", "device_plan", "model", "output"):
        parser.add_argument(name)
    parser.add_argument(
        "--sampler",
        default="spacemit_manipulation.charuco_sampler",
        help="Python module providing create_sampler()",
    )
    parser.add_argument("--option", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--count", type=int, default=12)
    args = parser.parse_args()
    if not 12 <= args.count <= 100:
        parser.error("count must be 12..100")
    config = load(args.model)
    model_path = Path(args.model).parent / config["urdf"]
    model = UrdfKinematics(model_path, config["base"], config["tip"], config["joints"])
    sampler = importlib.import_module(args.sampler).create_sampler(options(args.option))
    try:
        output = Path(args.output).resolve()
        output.mkdir(parents=True, exist_ok=False)
        samples = []
        for index in range(args.count):
            answer = input(
                f"样本 {index + 1}/{args.count}：调整姿态并停止其他控制器，"
                "确认静止/交接后输入 CAPTURE："
            )
            if answer != "CAPTURE":
                raise RuntimeError(
                    "collection aborted; partial evidence is retained, no dataset.json emitted"
                )
            # Release the device between samples.
            with DeviceClient(args.executable, args.device_plan, allow_motion=False) as client:
                record = sampler.capture(FeedbackPort(client, model), output / f"sample_{index:04}")
            record["joint_recording_source"] = "spacemit_cpp_distinct_feedback_urdf_fk"
            save_new(output / f"sample_{index:04}.json", record)
            samples.append(
                dict(
                    T_base_flange=[v for row in record["T_base_flange"] for v in row],
                    T_camera_target=[v for row in record["T_camera_board"] for v in row],
                )
            )
        save_new(
            output / "dataset.json", dict(schema_version=1, setup="eye_to_hand", samples=samples)
        )
        dependencies = [
            Path(args.device_plan),
            Path(args.model),
            model_path,
            *sampler.dependencies(),
        ]
        save_new(
            output / "provenance.json",
            dict(
                sampler.provenance(),
                sampler=args.sampler,
                sha256={
                    str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in dependencies
                },
            ),
        )
    finally:
        sampler.close()


if __name__ == "__main__":
    main()
