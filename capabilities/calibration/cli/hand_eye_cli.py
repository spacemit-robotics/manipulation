# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Eye-to-hand calibration commands; none moves the arm. Publication is quality-gated."""

import json
import os
from pathlib import Path
import subprocess
import sys
from .operator_support import Context, argument_parser, run
from .workbench import export_legacy, promote, save_new

SAMPLER = "spacemit_manipulation.charuco_sampler"


def add_sampler(command):
    command.add_argument(
        "--sampler", default=SAMPLER, help=f"camera sampler module (default: {SAMPLER})"
    )
    command.add_argument(
        "--option",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="sampler option, e.g. board=board.json serial=346222071954",
    )


def main():
    parser = argument_parser(__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    configure = commands.add_parser(
        "configure", help="create a device-independent collection config"
    )
    configure.add_argument("output", help="new JSON configuration")
    configure.add_argument("--plan", help="device configuration (default: SITE/device.yaml)")
    configure.add_argument("--installation-id", required=True)
    configure.add_argument("--board", required=True, help="ChArUco board JSON")
    configure.add_argument("--serial", help="camera serial; required with several cameras")
    collect = commands.add_parser(
        "collect", help="capture poses placed by the operator; no automatic motion"
    )
    collect.add_argument("name")
    collect.add_argument("--count", type=int, default=12)
    collect.add_argument("--plan", help="device configuration (default: SITE/device.yaml)")
    collect.add_argument("--config", help="shared calibration configuration; paths relative to it")
    add_sampler(collect)
    solve = commands.add_parser("solve", help="solve and quality-check dataset.json")
    solve.add_argument("name", help="dataset directory, relative to --site")
    solve.add_argument(
        "--max-error-mm", type=float, default=5.0, help="quality gate, at most 15 (default 5)"
    )
    export = commands.add_parser("export-perceptive", help="write a new perceptive_grasp fragment")
    export.add_argument("name", help="dataset directory, relative to --site")
    export.add_argument("--installation-id", required=True)
    export.add_argument(
        "--camera-type", choices=("realsense", "spacemit_las2"), default="realsense"
    )
    export.add_argument("--output", required=True, help="new output YAML/JSON fragment")
    solve.add_argument(
        "--max-error-deg", type=float, default=2.0, help="quality gate, at most 5 (default 2)"
    )
    for name, text in (
        ("promote", "copy a quality-passed candidate to active.json"),
        ("export", "write the application extrinsic.json"),
        ("restore", "verify active.json into a new restored.json"),
    ):
        command = commands.add_parser(name, help=text)
        command.add_argument("name", help="dataset directory, relative to --site")
    args = parser.parse_args()
    context = Context(args)
    device = context.binary("manipulation-device")
    if args.operation == "configure":
        output = Path(args.output).resolve()
        plan = context.plan(args.plan)
        model = context.model(plan)
        board = Path(args.board).resolve()
        if not board.is_file():
            raise FileNotFoundError(str(board))
        if not args.installation_id.strip():
            parser.error("--installation-id must not be blank")
        options = {"board": os.path.relpath(board, output.parent)}
        if args.serial:
            options["serial"] = args.serial
        save_new(
            output,
            {
                "schema_version": 1,
                "installation_id": args.installation_id,
                "device_plan": os.path.relpath(plan.resolve(), output.parent),
                "model": os.path.relpath(model.resolve(), output.parent),
                "sampler": {"module": SAMPLER, "options": options},
            },
        )
        print("calibration configuration:", output)
    elif args.operation == "collect":
        if args.config:
            if args.plan or args.option or args.sampler != SAMPLER:
                parser.error("--config selects the device and sampler; do not override them")
            context.module(
                "spacemit_manipulation.collect",
                "--config",
                args.config,
                "--device-executable",
                device,
                "--output",
                context.local(args.name),
                "--count",
                str(args.count),
            )
            return 0
        plan = context.plan(args.plan)
        arguments = [
            device,
            plan,
            context.model(plan),
            context.local(args.name),
            "--sampler",
            args.sampler,
            "--count",
            str(args.count),
        ]
        for item in args.option:
            arguments += ["--option", item]
        context.module("spacemit_manipulation.collect", *arguments)
    elif args.operation == "solve":
        directory = context.local(args.name)
        output = directory / "candidate.json"
        if output.exists():
            raise FileExistsError(str(output))
        if (directory / "manifest.json").exists():
            from .calibration_session import solve_session

            report = solve_session(directory, args.max_error_mm / 1000, args.max_error_deg)
            save_new(output, report)
            print(json.dumps(report), flush=True)
            print("report:", output)
            return 0 if report["quality_passed"] else 1
        result = subprocess.run(
            [
                str(context.binary("manipulation-calibration")),
                "solve",
                str(directory / "dataset.json"),
                "--max-error-mm",
                str(args.max_error_mm),
                "--max-error-deg",
                str(args.max_error_deg),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.stderr:
            print(result.stderr, file=sys.stderr, end="")
        if result.returncode not in (0, 1):
            return result.returncode
        report = json.loads(result.stdout)
        save_new(output, report)
        print(json.dumps(report), flush=True)
        print("report:", output)
        return result.returncode
    elif args.operation == "promote":
        promote(
            context.local(args.name) / "candidate.json", context.local(args.name) / "active.json"
        )
        print("quality-passed report:", context.local(args.name) / "active.json")
    elif args.operation == "export-perceptive":
        from .calibration_session import export_perceptive
        from .workbench import load

        directory = context.local(args.name)
        export_perceptive(
            load(directory / "active.json"), args.output, args.camera_type, args.installation_id
        )
        print("perceptive_grasp calibration fragment:", args.output)
    elif args.operation == "export":
        directory = context.local(args.name)
        export_legacy(
            directory / "active.json", directory / "provenance.json", directory / "extrinsic.json"
        )
        print("application extrinsic:", directory / "extrinsic.json")
    elif args.operation == "restore":
        promote(
            context.local(args.name) / "active.json", context.local(args.name) / "restored.json"
        )
        print("verified copy:", context.local(args.name) / "restored.json")


if __name__ == "__main__":
    raise SystemExit(run(main))
