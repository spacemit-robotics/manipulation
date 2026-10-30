# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""No-motion sampling, immutable teaching, explicit extrinsic promotion and restore."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from spacemit_manipulation import DeviceClient
from spacemit_manipulation.urdf_fk import UrdfKinematics, validate_transform, identity


def load(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def save_new(path, value):
    """Atomic exclusive publication: readers cannot observe half a JSON document."""
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix="." + path.name, delete=False) as stream:
            temporary = stream.name
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)  # Atomic, no replacement. Temporary and output share a filesystem.
    finally:
        if temporary is not None:
            os.unlink(temporary)


def sample(client, model, observe):
    """Bracket one trusted camera observation with fresh stationary arm feedback.

    observe() returns {T_camera_target:[16], captured_monotonic_ns:int} from a live
    camera adapter, not an arbitrary historical file. This function never moves.
    """
    before = client.state()
    observation = observe()
    after = client.state()
    age = (time.monotonic_ns() - observation["captured_monotonic_ns"]) / 1e9
    if not 0 <= age <= 0.2 or any(state["moving"] or state["fault"] for state in (before, after)):
        raise ValueError("sampling requires a fresh observation and stationary healthy arm")
    if len(before["joints_rad"]) != len(after["joints_rad"]) or any(
            abs(a - b) > 0.002 for a, b in zip(before["joints_rad"], after["joints_rad"])):
        raise ValueError("arm changed during capture")
    target = observation["T_camera_target"]
    if len(target) != 16 or not all(math.isfinite(value) for value in target):
        raise ValueError("invalid target transform")
    validate_transform(target)
    return dict(T_base_flange=model.forward(after["joints_rad"], check_limits=False), T_camera_target=target)


def validate_report(report):
    if (report.get("schema_version") != 1 or report.get("quality_passed") is not True or
            report.get("setup") != "eye_to_hand"):
        raise ValueError("only a quality-passed eye-to-hand candidate can be promoted")
    transform = report.get("T_base_camera", [])
    validate_transform(transform)
    # The gate recorded by the solver is re-checked here; it may never exceed 15 mm / 5 degrees.
    translation = report.get("threshold_translation_m", 0.005)
    rotation = report.get("threshold_rotation_rad", math.radians(2))
    for value, ceiling in ((translation, 0.015), (rotation, math.radians(5))):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= ceiling + 1e-12:
            raise ValueError("candidate quality threshold outside the allowed range")
    for key, limit in (("holdout_translation_m", translation), ("holdout_rotation_rad", rotation),
                       ("maximum_translation_m", translation), ("maximum_rotation_rad", rotation)):
        value = report.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= limit:
            raise ValueError("candidate residual fails quality gate")


def promote(candidate, destination):
    """Explicitly create a NEW active file; existing active files require operator-managed rename."""
    report = load(candidate)
    validate_report(report)
    save_new(destination, dict(report, candidate_sha256=hashlib.sha256(Path(candidate).read_bytes()).hexdigest()))


def export_legacy(candidate, provenance, destination):
    """Format conversion, not implicit activation; camera identity remains mandatory."""
    report, metadata = load(candidate), load(provenance)
    validate_report(report)
    pose = report["T_base_camera"]
    validate_transform(pose)
    camera = metadata["camera"]
    if camera.get("frame") != "color_optical" or not camera.get("serial"):
        raise ValueError("camera identity missing from collection provenance")
    unit = identity()
    save_new(destination, dict(schema=1, mode="eye_to_hand", quality_passed=True,
        direction="T_base_camera maps camera optical coordinates into arm base",
        T_base_camera=[pose[i:i+4] for i in range(0, 16, 4)],
        T_flange_tcp=[unit[i:i+4] for i in range(0, 16, 4)],
        camera=camera, board=metadata.get("board"), calibration_tcp="flange",
        quality_thresholds=dict(position_mm=report.get("threshold_translation_m", 0.005) * 1000,
                                angle_deg=math.degrees(report.get("threshold_rotation_rad", math.radians(2)))),
        source_candidate_sha256=hashlib.sha256(Path(candidate).read_bytes()).hexdigest()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    teach = commands.add_parser("teach", help="read current joints/hand into a new named pose file")
    teach.add_argument("executable")
    teach.add_argument("plan")
    teach.add_argument("output")
    apply = commands.add_parser("promote", help="quality-gated exclusive create; never reloads a running app")
    apply.add_argument("candidate")
    apply.add_argument("output")
    restore = commands.add_parser("restore", help="verify a promoted backup and copy into a new output")
    restore.add_argument("backup")
    restore.add_argument("output")
    export = commands.add_parser("export-legacy", help="write a new legacy-compatible camera extrinsic")
    export.add_argument("candidate")
    export.add_argument("provenance")
    export.add_argument("output")
    fk = commands.add_parser("fk", help="offline URDF FK; model JSON supplies base,tip,joints,urdf")
    fk.add_argument("model")
    fk.add_argument("pose")
    ack = commands.add_parser("ack-grip", help="publish this operation's explicit human grip declaration")
    ack.add_argument("token")
    ack.add_argument("output")
    ack.add_argument("--operator", required=True)
    args = parser.parse_args()
    if args.operation == "teach":
        with DeviceClient(args.executable, args.plan, allow_motion=False) as client:
            state = client.state()
            if state["moving"] or state["fault"]:
                raise ValueError("teach requires stationary healthy feedback")
            save_new(args.output, dict(schema_version=1, captured_epoch_s=time.time(), state=state))
    elif args.operation == "ack-grip":
        if (len(args.token) != 32 or any(ch not in "0123456789abcdef" for ch in args.token) or
                Path(args.output).stem != args.token or not args.operator.strip()):
            raise ValueError("use the exact current token/output from the application event")
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        save_new(args.output, dict(token=args.token, operator=args.operator, checked_epoch_s=time.time()))
    elif args.operation == "fk":
        config = load(args.model)
        model = UrdfKinematics(Path(args.model).parent / config["urdf"],
                              config["base"], config["tip"], config["joints"])
        print(json.dumps(model.forward(load(args.pose)["state"]["joints_rad"])))
    elif args.operation == "export-legacy":
        export_legacy(args.candidate, args.provenance, args.output)
    else:
        promote(args.candidate if args.operation == "promote" else args.backup, args.output)


if __name__ == "__main__":
    main()
