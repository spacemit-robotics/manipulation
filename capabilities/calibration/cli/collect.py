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
import json
from pathlib import Path
from spacemit_manipulation import DeviceClient
from spacemit_manipulation.sampling import FeedbackPort
from spacemit_manipulation.urdf_fk import UrdfKinematics
from spacemit_manipulation.workbench import load, save_new
from spacemit_manipulation.calibration_session import (
    CalibrationSession,
    device_identity,
    load_config,
    session_identity,
)


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
        parser.add_argument(name, nargs="?")
    parser.add_argument("--config", help="shared collection configuration with relative resources")
    parser.add_argument("--device-executable", help="read-only manipulation-device executable")
    parser.add_argument("--output", dest="configured_output")
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
    session_config = None
    if args.config:
        if any((args.executable, args.device_plan, args.model, args.output, args.option)):
            parser.error("--config cannot be combined with positional resources or --option")
        if not args.device_executable or not args.configured_output:
            parser.error("--config requires --device-executable and --output")
        session_config = load_config(args.config)
        args.executable, args.output = args.device_executable, args.configured_output
        args.device_plan, args.model = session_config["device_plan"], session_config["model"]
        args.sampler = session_config["sampler"]["module"]
        sampler_options = session_config["sampler"]["options"]
    else:
        if not all((args.executable, args.device_plan, args.model, args.output)):
            parser.error("specify executable device_plan model output, or --config")
        sampler_options = options(args.option)
    config = load(args.model)
    model_path = Path(args.model).parent / config["urdf"]
    model = UrdfKinematics(model_path, config["base"], config["tip"], config["joints"])
    output = Path(args.output).resolve()
    if output.exists():
        raise FileExistsError(str(output))
    sampler = importlib.import_module(args.sampler).create_sampler(sampler_options)
    try:
        session = None
        dependencies = [
            Path(args.device_plan),
            Path(args.model),
            model_path,
            *sampler.dependencies(),
        ]
        provenance = dict(
            sampler.provenance(),
            sampler=args.sampler,
            sha256={
                str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in dependencies
            },
        )
        provenance = json.loads(json.dumps(provenance, allow_nan=False))

        def check_resources():
            if any(
                hashlib.sha256(path.read_bytes()).hexdigest()
                != provenance["sha256"][str(path.resolve())]
                for path in dependencies
            ):
                raise ValueError("Calibration resources changed during collection")
            current = sampler.provenance()
            if current.get("camera") != provenance["camera"]:
                raise ValueError("Camera identity or intrinsics changed during collection")
            if current.get("board") != provenance.get("board"):
                raise ValueError("Board geometry changed during collection")

        if session_config:
            device = device_identity(
                args.device_plan, args.model, installation_id=session_config["installation_id"]
            )
            session = CalibrationSession(
                output, session_identity(device, provenance, session_config["frames"]), provenance
            )
        else:
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
            check_resources()
            # Release the device between samples.
            with DeviceClient(args.executable, args.device_plan, allow_motion=False) as client:
                record = sampler.capture(FeedbackPort(client, model), output / f"sample_{index:04}")
            record["joint_recording_source"] = "spacemit_cpp_distinct_feedback_urdf_fk"
            # Recheck resources between handoffs, before accepting another observation.
            check_resources()
            if session:
                session.append(record)
            else:
                save_new(output / f"sample_{index:04}.json", record)
            samples.append(
                dict(
                    T_base_flange=[v for row in record["T_base_flange"] for v in row],
                    T_camera_target=[v for row in record["T_camera_board"] for v in row],
                )
            )
        if session:
            session.finish()
        else:
            save_new(
                output / "dataset.json",
                dict(schema_version=1, setup="eye_to_hand", samples=samples),
            )
            save_new(output / "provenance.json", provenance)
    finally:
        sampler.close()


if __name__ == "__main__":
    main()
