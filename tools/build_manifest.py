# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Record source/install fingerprints and toolchain without touching hardware or Git state."""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

SOURCES = (
    "components/control/manipulation",
    "components/thirdparty/pyagxarm",
    "application/native/dice_arena",
)
EXCLUDE = {".git", "__pycache__", ".ruff_cache", ".pytest_cache"}


def fingerprint(root, paths):
    result = {}
    for relative in paths:
        for path in sorted((root / relative).rglob("*")):
            if path.is_file() and not EXCLUDE.intersection(path.relative_to(root).parts):
                result[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def command(*arguments):
    result = subprocess.run(arguments, text=True, capture_output=True, check=False, timeout=10)
    return dict(
        exit_code=result.returncode, output=result.stdout.strip(), error=result.stderr.strip()
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdk_root")
    parser.add_argument("build_root")
    parser.add_argument("output")
    parser.add_argument(
        "--external", action="append", default=[], help="name=external_git_worktree"
    )
    args = parser.parse_args()
    sdk, build = Path(args.sdk_root).resolve(), Path(args.build_root).resolve()
    for relative in SOURCES:
        if not (sdk / relative).is_dir():
            raise ValueError("source directory missing: " + relative)
    external = {}
    for item in args.external:
        name, path = item.split("=", 1)
        if name in external:
            raise ValueError("duplicate external name")
        external[name] = dict(
            path=str(Path(path).resolve()),
            head=command("git", "-C", path, "rev-parse", "HEAD"),
            status=command("git", "-C", path, "status", "--porcelain"),
        )
    report = dict(
        schema_version=1,
        created_epoch_s=time.time(),
        machine=platform.machine(),
        python=sys.version,
        compiler=command("c++", "--version"),
        cmake=command("cmake", "--version"),
        sdk_root=str(sdk),
        build_root=str(build),
        source_sha256=fingerprint(sdk, SOURCES),
        installed_sha256=fingerprint(build, ["install"]),
        external=external,
        vendor_commit=(sdk / "components/thirdparty/pyagxarm/UPSTREAM_COMMIT").read_text().strip(),
    )
    with Path(args.output).open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(
        json.dumps(
            dict(
                source_files=len(report["source_sha256"]),
                installed_files=len(report["installed_sha256"]),
                machine=report["machine"],
                output=str(Path(args.output).resolve()),
            )
        )
    )


if __name__ == "__main__":
    main()
