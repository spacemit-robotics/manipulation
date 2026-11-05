#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Create a device configuration from an installed model template without opening a device.

Templates live in descriptions/robots/<model>/devices/<variant>.yaml. Placeholders:
${datadir} ${libexecdir} ${bindir}: installation directories, written relative to the output;
${python}: interpreter command; ${name} / ${name:-default}: values from --set name=value.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

PLACEHOLDER = re.compile(r"\$\{([a-z_][a-z0-9_]*)(?::-([^}]*))?\}")


def templates(prefix, layout, model):
    directory = prefix / layout["datadir"] / "manipulation/descriptions/robots" / model / "devices"
    if not directory.is_dir():
        available = sorted(path.parent.name for path in
                           (prefix / layout["datadir"] / "manipulation/descriptions/robots").glob("*/devices")
                           if path.is_dir())
        raise ValueError(f"no device templates for model {model!r}; available: {', '.join(available) or 'none'}")
    return directory


def configure(prefix, model, output, variant="arm", values=None, python="python3"):
    prefix, output = Path(prefix).resolve(), Path(output).absolute()
    layout = json.loads((prefix / ".spacemit-manipulation.json").read_text())
    directory = templates(prefix, layout, model)
    template = directory / (variant + ".yaml")
    if not template.is_file():
        variants = sorted(path.stem for path in directory.glob("*.yaml"))
        raise ValueError(f"model {model!r} has no variant {variant!r}; available: {', '.join(variants)}")
    # Explicit interpreter paths are kept as given so a virtual environment stays selected.
    if "/" in python:
        python = str(Path(python).absolute())
    known = dict(values or {}, python=python)
    for key in ("datadir", "libexecdir", "bindir"):
        known[key] = os.path.relpath(prefix / layout[key], output.parent)

    def substitute(match):
        name, default = match.group(1), match.group(2)
        if name in known:
            return known[name]
        if default is not None:
            return default
        raise ValueError(f"template value required: --set {name}=...")
    content = PLACEHOLDER.sub(substitute, template.read_text())
    driver = re.search(r"(?m)^\s*driver:\s*(\S+)", content)
    binary = prefix / layout["bindir"] / "manipulation-device"
    drivers = subprocess.run([str(binary), "drivers"], check=True, capture_output=True, text=True).stdout.split()
    if not driver or driver.group(1) not in drivers:
        raise ValueError(f"this installation does not include driver {driver.group(1) if driver else '?'}; "
                         "rebuild with the matching driver module enabled")
    if output.exists():
        raise FileExistsError(f"{output} already exists; select it with `manipulation use {model}`, "
                              "or move it aside to configure again")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        stream.write(content)
    result = subprocess.run([str(binary), "validate", str(output)], capture_output=True, text=True)
    if result.returncode:
        output.unlink()
        raise ValueError("generated configuration is invalid: " + result.stderr.strip())
    print(f"created {output} from {model}/{variant}; no hardware accessed")


def parse_values(items):
    values = {}
    for item in items:
        name, separator, value = item.partition("=")
        if not separator or not re.fullmatch(r"[a-z_][a-z0-9_]*", name):
            raise ValueError(f"expected NAME=VALUE, got {item!r}")
        values[name] = value
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix")
    parser.add_argument("model")
    parser.add_argument("output")
    parser.add_argument("--variant", default="arm")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--python", default="python3")
    args = parser.parse_args()
    configure(args.prefix, args.model, args.output, args.variant, parse_values(args.set), args.python)


if __name__ == "__main__":
    main()
