# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Installation-relative command support; no device construction or application policy."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys


def default_site():
    """Site files never depend on the working directory: MANIPULATION_SITE or the user data dir."""
    if os.environ.get("MANIPULATION_SITE"):
        return os.environ["MANIPULATION_SITE"]
    data = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local/share")
    return str(Path(data) / "spacemit-manipulation/site")


def argument_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--prefix", required=True, help=argparse.SUPPRESS)
    parser.add_argument("--site", default=default_site(),
                        help="site directory holding one subdirectory per device and the current device "
                             "(default: $MANIPULATION_SITE or ~/.local/share/spacemit-manipulation/site)")
    return parser


class Context:
    def __init__(self, args):
        self.prefix = Path(args.prefix).resolve()
        self.site = Path(args.site).expanduser().absolute()
        self.layout = json.loads((self.prefix / ".spacemit-manipulation.json").read_text())
        if self.layout.get("schema_version") != 1:
            raise ValueError("unsupported installation layout")
        for key in ("bindir", "libdir", "datadir", "libexecdir", "pythondir"):
            path = Path(self.layout[key])
            if path.is_absolute() or not (self.prefix / path).resolve().is_relative_to(self.prefix):
                raise ValueError("installation directory must remain inside the prefix: " + key)

    def binary(self, name):
        return self.prefix / self.layout["bindir"] / name

    def data(self, component):
        return self.prefix / self.layout["datadir"] / component

    def local(self, name):
        return self.site / name

    def plan(self, override=None):
        """The current device's configuration (SITE/device.yaml), or an explicit --plan."""
        plan = Path(override).absolute() if override else self.local("device.yaml")
        if plan.is_file():
            return plan
        if override:
            raise FileNotFoundError(f"{plan} not found")
        devices = self.devices()
        if devices:
            raise FileNotFoundError(f"no current device; choose one with `manipulation use NAME` "
                                    f"(configured: {', '.join(devices)})")
        raise FileNotFoundError("no device configured; create one with `manipulation configure MODEL`")

    def plan_for(self, model, override=None):
        """The current device, for an application written for one robot model."""
        plan = self.plan(override)
        actual = self.describe(plan)["model"]
        if actual != model:
            raise ValueError(f"this application needs a {model} device, but {plan} is {actual or 'another device'}; "
                             f"select it with `manipulation use {model}`")
        return plan

    def devices(self):
        """Configured devices: SITE/<name>/device.yaml."""
        return sorted(path.parent.name for path in self.site.glob("*/device.yaml") if path.is_file())

    def current(self):
        """The current device's name, or None."""
        link = self.local("device.yaml")
        if link.is_symlink() and link.is_file():
            return link.resolve().parent.name
        return None

    def check_selectable(self):
        """SITE/device.yaml must be the selection link, never a configuration of its own."""
        link = self.local("device.yaml")
        if link.exists() and not link.is_symlink():
            raise FileExistsError(f"{link} is a plain file, not a device selection; "
                                  "move it aside and configure the device again")

    def select(self, name):
        """Make SITE/<name>/device.yaml the current device: SITE/device.yaml links to it."""
        self.check_selectable()
        if name not in self.devices():
            raise FileNotFoundError(f"no device {name!r}; configured: {', '.join(self.devices()) or 'none'}")
        temporary = self.local(".device.yaml.new")
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(Path(name) / "device.yaml")
        os.replace(temporary, self.local("device.yaml"))

    def device_file(self, plan, name):
        """A file kept beside a device configuration, such as a recorded pose."""
        return Path(plan).resolve().parent / name

    def describe(self, plan):
        """Driver, model, joint and channel names, parsed by the C++ loader. Never opens a device."""
        result = subprocess.run([str(self.binary("manipulation-device")), "describe", str(plan)],
                                capture_output=True, text=True, check=False)
        if result.returncode:
            raise ValueError(result.stderr.strip() or "invalid device configuration")
        return json.loads(result.stdout)

    def model(self, plan):
        """Kinematic model description for the plan's `device.model`."""
        name = self.describe(plan)["model"]
        if not name:
            raise ValueError(f"{plan} has no device.model; kinematic tools need a robot description")
        model = self.data("manipulation") / "descriptions/robots" / name / "model.json"
        if not model.is_file():
            raise FileNotFoundError(f"robot description {name!r} has no model.json")
        return model

    def environment(self):
        environment = dict(os.environ)
        modules = str(self.prefix / self.layout["pythondir"])
        environment["PYTHONPATH"] = modules + (os.pathsep + environment["PYTHONPATH"]
                                               if environment.get("PYTHONPATH") else "")
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        return environment

    def execute(self, arguments):
        """Replace the launcher so signals reach the existing control executable directly."""
        arguments = [str(argument) for argument in arguments]
        os.execve(arguments[0], arguments, self.environment())

    def script(self, path, *arguments):
        self.execute([sys.executable, path, *arguments])

    def module(self, name, *arguments):
        self.execute([sys.executable, "-m", name, *arguments])


def load_tool(path):
    spec = importlib.util.spec_from_file_location("_operator_" + Path(path).stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(main):
    try:
        return main() or 0
    except KeyboardInterrupt:
        print("cancel requested; inspect the physical device before restarting", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, EOFError,
            subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        return 2
