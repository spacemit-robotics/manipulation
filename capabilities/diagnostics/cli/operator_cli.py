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


def hand_grasp(context, args):
    """Close until the hand stops on an object (holding) or reaches the target (empty)."""
    import json
    from .client import DeviceClient
    plan = context.plan(args.plan)
    channels = context.describe(plan)["channels"]
    if not channels:
        raise ValueError(f"{plan} has no end effector")
    target = args.positions if args.positions is not None else [0.0] * len(channels)
    if len(target) != len(channels) or not all(0 <= value <= 1 for value in target):
        raise ValueError(f"give {len(channels)} positions in [0,1] in channel order: {' '.join(channels)}")
    with DeviceClient(context.binary("manipulation-device"), plan) as client:
        capabilities = client.capabilities()["effector"]
        if not capabilities or not capabilities.get("grasp_detection"):
            raise ValueError("this end effector reports no grasp evidence; use `manipulation hand-move`")
        result = client.move_effector(target, duration_s=args.duration)
        state = client.state()
        print(json.dumps(dict(result, holding=state["effector_holding"], loads=state["effector_loads"],
                              positions=state["effector_positions"])), flush=True)


def monitor(context, args):
    """Print fresh state until Ctrl+C (or --count samples). Read only: with torque released the arm and
    hand can be moved by hand meanwhile, as in the former test tool's monitor."""
    import math
    import time
    from .client import DeviceClient
    if not 1 <= args.rate <= 20 or args.count < 0:
        raise ValueError("--rate must be 1..20 Hz and --count nonnegative")
    plan = context.plan(args.plan)
    channels = context.describe(plan)["channels"]
    with DeviceClient(context.binary("manipulation-device"), plan, allow_motion=False) as client:
        started = time.monotonic()
        sample = 0
        try:
            while not args.count or sample < args.count:
                state = client.state()
                fields = [f"{time.monotonic() - started:6.1f}s", "joints_deg",
                          " ".join(f"{math.degrees(value):7.1f}" for value in state["joints_rad"]),
                          "enabled" if state["enabled"] else "released",
                          "moving" if state["moving"] else "still", "FAULT" if state["fault"] else "ok"]
                fields += [f"{name} {value:.3f}" for name, value in
                           zip(channels, state.get("effector_positions") or [])]
                if state.get("effector_loads"):
                    fields.append("load " + " ".join(f"{value:+.2f}" for value in state["effector_loads"]))
                if state.get("effector_holding") is not None:
                    fields.append("holding" if state["effector_holding"] else "not holding")
                print("  ".join(fields), flush=True)
                sample += 1
                time.sleep(max(0.0, started + sample / args.rate - time.monotonic()))
        except KeyboardInterrupt:  # The normal end of monitoring; nothing was commanded.
            print(flush=True)


def load_model(context, plan):
    """The device model's kinematic chain (model.json of `device.model`)."""
    import json
    from .urdf_fk import UrdfKinematics
    model = context.model(plan)
    config = json.loads(model.read_text(encoding="utf-8"))
    return UrdfKinematics(model.parent / config["urdf"], config["base"], config["tip"], config["joints"])


def check_duration(state, target, speed, timeout_s):
    """Refuse a point move that cannot finish within the arm timeout even at the full commanded speed."""
    limits = state.get("controller_limits") or []
    if len(limits) != len(target):
        return
    needed = max((abs(goal - joint) / (speed * limit["max_velocity_rad_s"])
                  for joint, goal, limit in zip(state["joints_rad"], target, limits)
                  if limit["max_velocity_rad_s"] > 0), default=0)
    if needed > timeout_s:
        raise ValueError(f"the move needs at least {needed:.0f} s at --speed {speed:g}, more than the arm "
                         f"timeout_s {timeout_s:g}; raise --speed or move in shorter steps; nothing moved")


def kinematics(context, args):
    """Tip poses in the base frame: position in metres, URDF roll-pitch-yaw in radians."""
    import json
    from .client import DeviceClient
    from .urdf_ik import line, matrix, pose, rpy, solve
    plan = context.plan(args.plan)
    model = load_model(context, plan)
    if args.operation == "pose":
        with DeviceClient(context.binary("manipulation-device"), plan, allow_motion=False) as client:
            joints = client.state()["joints_rad"]
        tip = matrix(model.forward(joints, check_limits=False))
        print(json.dumps(dict(joints_rad=joints, position_m=tip[:3, 3].tolist(), rpy_rad=rpy(tip[:3, :3]))))
        return
    target = pose(args.position, args.rpy)
    weight = args.orientation_weight if args.orientation_weight is not None else (1.0 if args.rpy else 0.0)
    described = context.describe(plan)
    limits = described["joint_limits_rad"]
    if args.operation == "ik" and args.seed_pose:  # Offline: the seed comes from a saved pose.
        seed = json.loads(context.local(args.seed_pose).read_text(encoding="utf-8"))["state"]["joints_rad"]
        result = solve(model, target, seed, orientation_weight=weight, limits=limits)
        print(json.dumps(result))
        if not result["converged"]:
            raise ValueError("no joint solution within tolerance from this seed")
        return
    with DeviceClient(context.binary("manipulation-device"), plan, allow_motion=args.operation != "ik") as client:
        state = client.state()
        current = state["joints_rad"]
        if args.operation == "line-move":
            waypoints = line(model, current, target, step_m=args.step_mm / 1000, orientation_weight=weight,
                             limits=limits)
            print(f"{len(waypoints)} waypoints", flush=True)
            if client.capabilities()["arm"]["joint_path"]:
                for start in range(0, len(waypoints), 128):  # The device accepts 128 waypoints per path.
                    result = client.move_arm_path(waypoints[start:start + 128], args.speed, args.path_velocity,
                                                  args.path_acceleration)
                print(result)
            else:  # Point moves only: hand over within tolerance, stop at the last waypoint.
                for waypoint in waypoints[:-1]:
                    client.move_arm(waypoint, args.speed, wait_for_arrival=False)
                print(client.move_arm(waypoints[-1], args.speed))
            return
        result = solve(model, target, current, orientation_weight=weight, limits=limits)
        print(json.dumps(result), flush=True)
        if not result["converged"]:
            raise ValueError("no joint solution within tolerance from the current pose; nothing moved")
        if args.operation == "pose-move":
            check_duration(state, result["joints_rad"], args.speed, described["arm_timeout_s"])
            print(client.move_arm(result["joints_rad"], args.speed))


def device_action(context, args):
    """Only the explicitly selected torque operation receives its required permission."""
    from .client import DeviceClient
    actions = {
        "enable": ("enable_arm", "allow_enable"),
        "disable": ("disable_arm", "allow_disable"),
        "hand-enable": ("enable_effector", "allow_effector_enable"),
        "hand-disable": ("disable_effector", "allow_effector_disable"),
        "hold": ("hold_arm", None),
        "hand-hold": ("hold_effector", None),
    }
    method, permission = actions[args.operation]
    permissions = {permission: True} if permission else {}
    with DeviceClient(context.binary("manipulation-device"), context.plan(args.plan), **permissions) as client:
        print(getattr(client, method)(), flush=True)


def joint_move(context, args):
    """Absolute joint vector or one relative joint step; the runtime owns limit checks."""
    from .client import DeviceClient
    plan = context.plan(args.plan)
    joints = context.describe(plan)["joints"]
    if args.operation == "joint-step":
        index = joints.index(args.joint) if args.joint in joints else (
            int(args.joint) if args.joint.isdigit() else -1)
        if not 0 <= index < len(joints):
            raise ValueError(f"joint must be 0..{len(joints) - 1} or one of {joints}")
    elif len(args.positions) != len(joints):
        raise ValueError(f"give {len(joints)} joint positions in radians, in order: {' '.join(joints)}")
    with DeviceClient(context.binary("manipulation-device"), plan) as client:
        if args.operation == "joint-step":
            result = client.jog_arm(index, args.delta, args.speed)
        else:
            result = client.move_arm(args.positions, args.speed)
        print(result, flush=True)


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
                       ("capabilities", "show supported device operations without opening hardware"),
                       ("arm-check", "bounded supervised arm motion check")):
        command = commands.add_parser(name, help=text)
        command.add_argument("--plan", help="device configuration (default: SITE/device.yaml)")
        if name == "arm-check":
            command.add_argument("--timeout-only", action="store_true")
    for name, text in (("enable", "enable arm torque"), ("disable", "release arm torque"),
                       ("hand-enable", "enable end-effector torque"),
                       ("hand-disable", "release end-effector torque"),
                       ("hold", "hold the arm without releasing torque"),
                       ("hand-hold", "hold the end effector without releasing torque")):
        command = commands.add_parser(name, help=text)
        command.add_argument("--plan")
    for name in ("joint-step", "joint-move"):
        command = commands.add_parser(name, help="move joints in radians using configured limits")
        if name == "joint-step":
            command.add_argument("joint", help="zero-based joint index or configured joint name")
            command.add_argument("delta", type=float, help="relative step in radians")
        else:
            command.add_argument("positions", type=float, nargs="+", help="one target in radians per joint")
        command.add_argument("--speed", type=float, default=0.05, help="controller speed scale in (0,1]")
        command.add_argument("--plan")
    move = commands.add_parser("hand-move", help="move every end-effector channel to an absolute position")
    move.add_argument("positions", type=float, nargs="+", help="one value in [0,1] per channel, in channel order")
    move.add_argument("--duration", type=float, default=2.0, help="execution time in seconds (default 2.0)")
    move.add_argument("--plan")
    hand = commands.add_parser("hand-step", help="move one end-effector channel by 0.01")
    hand.add_argument("channel", help="channel index or name")
    hand.add_argument("delta", type=float)
    hand.add_argument("--plan")
    grasp = commands.add_parser("hand-grasp", help="close until holding an object or fully closed (grasp detection)")
    grasp.add_argument("--positions", type=float, nargs="+", help="closing target per channel (default all 0)")
    grasp.add_argument("--duration", type=float, default=1.0, help="execution time in seconds (default 1.0)")
    grasp.add_argument("--plan")
    commands.add_parser("pose", help="current tip pose from the device model (read only)").add_argument("--plan")
    watch = commands.add_parser("monitor", help="print joint and end-effector state until Ctrl+C (read only)")
    watch.add_argument("--rate", type=float, default=5.0, help="samples per second, 1..20 (default 5)")
    watch.add_argument("--count", type=int, default=0, help="stop after N samples (default: until Ctrl+C)")
    watch.add_argument("--plan")
    for name, text in (("ik", "joint solution for a tip pose; no motion"),
                       ("pose-move", "move the tip to a pose by joint motion"),
                       ("line-move", "move the tip along a straight line")):
        command = commands.add_parser(name, help=text)
        command.add_argument("position", type=float, nargs=3, metavar=("X", "Y", "Z"),
                             help="tip position in the base frame, metres")
        command.add_argument("--rpy", type=float, nargs=3, metavar=("ROLL", "PITCH", "YAW"),
                             help="tip orientation, radians (default: position only)")
        command.add_argument("--orientation-weight", type=float,
                             help="0 position only, up to 1 full orientation (default 1 with --rpy, else 0)")
        command.add_argument("--plan")
        if name == "ik":
            command.add_argument("--seed-pose", help="solve offline from a saved pose (relative to --site)")
        else:
            command.add_argument("--speed", type=float, default=0.05, help="controller speed scale in (0,1]")
        if name == "line-move":
            command.add_argument("--step-mm", type=float, default=5.0, help="waypoint spacing (default 5 mm)")
            command.add_argument("--path-velocity", type=float, default=0.2,
                                 help="joint-path drivers: largest joint speed, rad/s (default 0.2)")
            command.add_argument("--path-acceleration", type=float, default=0.5,
                                 help="joint-path drivers: largest joint acceleration, rad/s^2 (default 0.5)")
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
    elif args.operation in ("validate", "inspect", "capabilities"):
        context.execute([device, args.operation, context.plan(args.plan)])
    elif args.operation in ("enable", "disable", "hand-enable", "hand-disable", "hold", "hand-hold"):
        device_action(context, args)
    elif args.operation in ("joint-step", "joint-move"):
        joint_move(context, args)
    elif args.operation == "arm-check":
        extra = ["--timeout-only"] if args.timeout_only else []
        context.execute([device, "arm-check", context.plan(args.plan), *extra])
    elif args.operation == "hand-step":
        hand_step(context, args)
    elif args.operation == "hand-move":
        hand_move(context, args)
    elif args.operation == "hand-grasp":
        hand_grasp(context, args)
    elif args.operation == "monitor":
        monitor(context, args)
    elif args.operation in ("pose", "ik", "pose-move", "line-move"):
        kinematics(context, args)
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
