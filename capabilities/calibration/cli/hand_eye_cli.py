# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Eye-to-hand calibration commands; none moves the arm. Publication is quality-gated."""
import json
import subprocess
import sys
from .operator_support import Context, argument_parser, run
from .workbench import export_legacy, promote, save_new

SAMPLER = "spacemit_manipulation.charuco_sampler"


def add_sampler(command):
    command.add_argument("--sampler", default=SAMPLER, help=f"camera sampler module (default: {SAMPLER})")
    command.add_argument("--option", action="append", default=[], metavar="NAME=VALUE",
                         help="sampler option, e.g. board=board.json serial=346222071954")


def main():
    parser = argument_parser(__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    collect = commands.add_parser("collect", help="capture poses placed by the operator; no automatic motion")
    collect.add_argument("name")
    collect.add_argument("--count", type=int, default=12)
    collect.add_argument("--plan", help="device configuration (default: SITE/device.yaml)")
    add_sampler(collect)
    solve = commands.add_parser("solve", help="solve and quality-check dataset.json")
    solve.add_argument("name", help="dataset directory, relative to --site")
    solve.add_argument("--max-error-mm", type=float, default=5.0, help="quality gate, at most 15 (default 5)")
    solve.add_argument("--max-error-deg", type=float, default=2.0, help="quality gate, at most 5 (default 2)")
    for name, text in (("promote", "copy a quality-passed candidate to active.json"),
                       ("export", "write the application extrinsic.json"),
                       ("restore", "verify active.json into a new restored.json")):
        command = commands.add_parser(name, help=text)
        command.add_argument("name", help="dataset directory, relative to --site")
    args = parser.parse_args()
    context = Context(args)
    device = context.binary("manipulation-device")
    if args.operation == "collect":
        plan = context.plan(args.plan)
        arguments = [device, plan, context.model(plan), context.local(args.name),
                     "--sampler", args.sampler, "--count", str(args.count)]
        for item in args.option:
            arguments += ["--option", item]
        context.module("spacemit_manipulation.collect", *arguments)
    elif args.operation == "solve":
        directory = context.local(args.name)
        output = directory / "candidate.json"
        if output.exists():
            raise FileExistsError(str(output))
        result = subprocess.run([str(context.binary("manipulation-calibration")), "solve",
                                 str(directory / "dataset.json"), "--max-error-mm", str(args.max_error_mm),
                                 "--max-error-deg", str(args.max_error_deg)],
                                capture_output=True, text=True, check=False)
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
        promote(context.local(args.name) / "candidate.json", context.local(args.name) / "active.json")
        print("quality-passed report:", context.local(args.name) / "active.json")
    elif args.operation == "export":
        directory = context.local(args.name)
        export_legacy(directory / "active.json", directory / "provenance.json", directory / "extrinsic.json")
        print("application extrinsic:", directory / "extrinsic.json")
    elif args.operation == "restore":
        promote(context.local(args.name) / "active.json", context.local(args.name) / "restored.json")
        print("verified copy:", context.local(args.name) / "restored.json")


if __name__ == "__main__":
    raise SystemExit(run(main))
