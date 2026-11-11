#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Check isolated simulation configuration and installed physics consumption."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def run(arguments, directory, environment):
    result = subprocess.run(
        [str(argument) for argument in arguments],
        cwd=directory,
        env=environment,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def cache(build, name):
    for line in (build / "CMakeCache.txt").read_text().splitlines():
        if line.startswith(name + ":"):
            return line.partition("=")[2]
    raise AssertionError(f"missing cache entry: {name}")


def configure(source, build, environment, *options):
    run(
        [
            "cmake", "-S", source, "-B", build,
            "-DBUILD_TESTING=OFF", "-DCMAKE_BUILD_TYPE=Release",
            "-DMANIPULATION_ENABLE_NERO=OFF", "-DMANIPULATION_ENABLE_SO101=OFF",
            "-DMANIPULATION_ENABLE_CALIBRATION=OFF",
            "-DMANIPULATION_ENABLE_MUJOCO=ON",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
            "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF", *options,
        ],
        source,
        environment,
    )


def check(prefix):
    with tempfile.TemporaryDirectory(prefix="simulation-build-") as temporary:
        work = Path(temporary)
        source = work / "sdk/components/control/manipulation"
        source.mkdir(parents=True)
        for name in (
            "CMakeLists.txt", "LICENSE", "NOTICE", "cmake", "core", "runtime", "include",
            "drivers", "capabilities", "bindings", "descriptions", "examples", "tools",
            "simulation",
        ):
            item = ROOT / name
            if item.is_dir():
                shutil.copytree(
                    item, source / name,
                    ignore=shutil.ignore_patterns("__pycache__", "assets"),
                )
            else:
                shutil.copy2(item, source / name)
        environment = dict(os.environ)
        for name in ("LD_LIBRARY_PATH", "PYTHONPATH", "CMAKE_PREFIX_PATH", "mujoco_world_DIR"):
            environment.pop(name, None)
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        bundled = source / "simulation/mujoco"
        old_default = source / "../../simulation/mujoco"
        assert not old_default.exists()

        # Disallow an installed package from hiding a missing bundled source tree.
        build = work / "local-build"
        disable_package = "-DCMAKE_DISABLE_FIND_PACKAGE_mujoco_world=ON"
        configure(source, build, environment, disable_package)
        assert Path(cache(build, "MANIPULATION_MUJOCO_SOURCE_DIR")) == bundled
        configure(
            source, build, environment, disable_package,
            f"-DMANIPULATION_MUJOCO_SOURCE_DIR={old_default}",
        )
        assert Path(cache(build, "MANIPULATION_MUJOCO_SOURCE_DIR")) == bundled

        external = work / "external-physics"
        shutil.copytree(bundled, external)
        with (external / "CMakeLists.txt").open("a") as stream:
            stream.write(
                '\noption(MUJOCO_BUILD_LEGACY_SIMULATOR "Build the humanoid simulator" ON)\n'
                "if(MUJOCO_BUILD_LEGACY_SIMULATOR)\n"
                "    find_package(LegacyHumanoidGraphics REQUIRED)\n"
                "endif()\n"
            )
        graphics_disabled = "-DCMAKE_DISABLE_FIND_PACKAGE_LegacyHumanoidGraphics=ON"
        standalone = subprocess.run(
            ["cmake", "-S", str(external), "-B", str(work / "standalone-external"),
             "-DBUILD_TESTING=OFF", graphics_disabled],
            cwd=work, env=environment, stdin=subprocess.DEVNULL,
            capture_output=True, text=True, timeout=300, check=False,
        )
        assert standalone.returncode != 0
        assert "LegacyHumanoidGraphics" in standalone.stderr
        configure(
            source, work / "external-build", environment, disable_package, graphics_disabled,
            f"-DMANIPULATION_MUJOCO_SOURCE_DIR={external}",
        )
        assert Path(cache(work / "external-build", "MANIPULATION_MUJOCO_SOURCE_DIR")) == external

        original = work / "install"
        shutil.copytree(prefix, original, symlinks=True)
        relocated = work / "relocated install"
        original.rename(relocated)
        missing = work / "absent-physics"
        configure(
            source, work / "package-build", environment,
            f"-DMANIPULATION_MUJOCO_SOURCE_DIR={missing}",
            f"-DCMAKE_PREFIX_PATH={relocated}",
        )
        package = Path(cache(work / "package-build", "mujoco_world_DIR"))
        assert relocated in package.parents

        configure(
            source, work / "remote-build", environment, disable_package,
            "-DMANIPULATION_ENABLE_MUJOCO=OFF", "-DMANIPULATION_ENABLE_MUJOCO_REMOTE=ON",
            f"-DMANIPULATION_MUJOCO_SOURCE_DIR={missing}",
        )
        assert Path(cache(work / "remote-build", "MANIPULATION_MUJOCO_SOURCE_DIR")) == missing

        consumer = work / "consumer"
        consumer.mkdir()
        shutil.copy2(bundled / "tests/world.xml", consumer / "world.xml")
        (consumer / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(physics_consumer LANGUAGES CXX)\n"
            "find_package(mujoco_world CONFIG REQUIRED)\n"
            "add_executable(physics-consumer main.cpp)\n"
            "target_link_libraries(physics-consumer PRIVATE mujoco_sim::world)\n"
            "if(TARGET mujoco_sim::render)\n"
            "  target_link_libraries(physics-consumer PRIVATE mujoco_sim::render)\n"
            "endif()\n"
        )
        (consumer / "main.cpp").write_text(
            '#include "mujoco_world.h"\n'
            "int main() {\n"
            '    mujoco_sim::World world("world.xml");\n'
            "    world.Step();\n"
            "    return world.Snapshot().IsValid() ? 0 : 1;\n"
            "}\n"
        )
        shutil.rmtree(source)
        shutil.rmtree(external)
        run(
            ["cmake", "-S", consumer, "-B", consumer / "build",
             f"-DCMAKE_PREFIX_PATH={relocated}", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"],
            consumer, environment,
        )
        run(["cmake", "--build", consumer / "build", "-j2"], consumer, environment)
        run([consumer / "build/physics-consumer"], consumer, environment)
    print("simulation build: isolated source, cache upgrade, overrides, remote and relocation passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    arguments = parser.parse_args()
    check(arguments.prefix.resolve())


if __name__ == "__main__":
    main()
