# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Device operator commands. Model-independent; no implicit enable, home or recovery."""
from .operator_support import Context, argument_parser, load_tool, run


def hand_step(context, args):
    plan = context.plan(args.plan)
    described = context.describe(plan)
    channels = described["channels"]
    if not channels:
        raise ValueError(f"{plan} has no end effector")
    if described["effector_feedback"] == "none":
        raise ValueError("this end effector has no position feedback, so a relative step is undefined; "
                         "use `manipulation hand-move` with absolute positions")
    index = channels.index(args.channel) if args.channel in channels else (
        int(args.channel) if args.channel.isdigit() else -1)
    if not 0 <= index < len(channels) or args.delta not in (-0.01, 0.01):
        raise ValueError(f"channel must be 0..{len(channels) - 1} or one of {channels}; delta -0.01 or 0.01")
    from .client import DeviceClient
    with DeviceClient(context.binary("manipulation-device"), plan) as client:
        state = client.state()
        print("current state:", state, flush=True)
        target = list(state["effector_positions"])
        target[index] += args.delta
        if not 0 <= target[index] <= 1:
            raise ValueError("target outside normalized range")
        print(f"target ({channels[index]}):", target, flush=True)
        print(client.move_effector(target, duration_s=1.0), flush=True)
        print(client.state())


def hand_move(context, args):
    """Absolute target for every channel. Without feedback the starting position is unknown."""
    plan = context.plan(args.plan)
    described = context.describe(plan)
    channels = described["channels"]
    if not channels:
        raise ValueError(f"{plan} has no end effector")
    if len(args.positions) != len(channels) or not all(0 <= value <= 1 for value in args.positions):
        raise ValueError(f"give {len(channels)} positions in [0,1] in channel order: {' '.join(channels)}")
    from .client import DeviceClient
    with DeviceClient(context.binary("manipulation-device"), plan) as client:
        state = client.state()
        if described["effector_feedback"] == "none":
            print("current position: unknown (no feedback); the hand may move by any amount", flush=True)
        else:
            print("current position:", state["effector_positions"], flush=True)
        print("target:", dict(zip(channels, args.positions)), f"duration {args.duration} s", flush=True)
        print(client.move_effector(args.positions, duration_s=args.duration, allow_unverified=True), flush=True)


def main():
    parser = argument_parser(__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    check = commands.add_parser("check", help="verify dependencies without opening hardware")
    check.add_argument("--profile", choices=("build", "control", "vision"), default="control")
    commands.add_parser("demo", help="run the Mock arm and hand point task; no hardware")
    configure = commands.add_parser("configure", help="create SITE/device.yaml from a model template")
    configure.add_argument("model", help="robot description, for example nero")
    configure.add_argument("--variant", default="arm", help="template under the model's devices/ (default: arm)")
    configure.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                           help="template value, for example channel=can1")
    configure.add_argument("--python", default="python3", help="interpreter for vendor workers")
    for name, text in (("validate", "check the configuration offline"), ("inspect", "connect and read; no motion"),
                       ("arm-check", "bounded supervised arm motion check")):
        command = commands.add_parser(name, help=text)
        command.add_argument("--plan", help="device configuration (default: SITE/device.yaml)")
        if name == "arm-check":
            command.add_argument("--timeout-only", action="store_true")
    move = commands.add_parser("hand-move", help="move every end-effector channel to an absolute position")
    move.add_argument("positions", type=float, nargs="+", help="one value in [0,1] per channel, in channel order")
    move.add_argument("--duration", type=float, default=2.0, help="execution time in seconds (default 2.0)")
    move.add_argument("--plan")
    hand = commands.add_parser("hand-step", help="move one end-effector channel by 0.01")
    hand.add_argument("channel", help="channel index or name")
    hand.add_argument("delta", type=float)
    hand.add_argument("--plan")
    for name, text in (("teach", "save the stationary state to a new SITE file"),
                       ("fk", "forward kinematics of a saved pose")):
        command = commands.add_parser(name, help=text)
        command.add_argument("pose", help="pose filename, relative to --site")
        command.add_argument("--plan")
    manifest = commands.add_parser("manifest", help="record source/install fingerprints without hardware")
    manifest.add_argument("--source", default=".")
    manifest.add_argument("--build", default="output/manipulation-dev")
    manifest.add_argument("--output", default="build_manifest.json")
    manifest.add_argument("--external", action="append", default=[])
    args = parser.parse_args()
    context = Context(args)
    device = context.binary("manipulation-device")
    if args.operation == "check":
        context.script(context.data("manipulation") / "tools/check_environment.py",
                       "--prefix", context.prefix, "--profile", args.profile)
    elif args.operation == "demo":
        context.execute([device, "run", context.data("manipulation") / "examples/mock_device.yaml"])
    elif args.operation == "configure":
        tool = load_tool(context.data("manipulation") / "tools/configure_device.py")
        tool.configure(context.prefix, args.model, context.local("device.yaml"), args.variant,
                       tool.parse_values(args.set), args.python)
    elif args.operation in ("validate", "inspect"):
        context.execute([device, args.operation, context.plan(args.plan)])
    elif args.operation == "arm-check":
        extra = ["--timeout-only"] if args.timeout_only else []
        context.execute([device, "arm-check", context.plan(args.plan), *extra])
    elif args.operation == "hand-step":
        hand_step(context, args)
    elif args.operation == "hand-move":
        hand_move(context, args)
    elif args.operation == "teach":
        if context.local(args.pose).exists():
            raise FileExistsError(str(context.local(args.pose)))
        context.module("spacemit_manipulation.workbench", "teach", device, context.plan(args.plan),
                       context.local(args.pose))
    elif args.operation == "fk":
        if not context.local(args.pose).is_file():
            raise FileNotFoundError(f"{context.local(args.pose)} not found; record one with `manipulation teach`")
        context.module("spacemit_manipulation.workbench", "fk", context.model(context.plan(args.plan)),
                       context.local(args.pose))
    elif args.operation == "manifest":
        arguments = [args.source, args.build, context.local(args.output)]
        for external in args.external:
            arguments += ["--external", external]
        context.script(context.data("manipulation") / "tools/build_manifest.py", *arguments)


if __name__ == "__main__":
    raise SystemExit(run(main))
