#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Check isolated simulation configuration and installed physics consumption."""

import argparse
import hashlib
import os
from pathlib import Path
import re
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


def check_gl4es(work, environment):
    """Exercise dependency selection and patching without network or GPU access."""
    work = work / "gl4es-contract"
    work.mkdir()
    modules = work / "cmake"
    shutil.copytree(ROOT / "simulation/mujoco/cmake", modules)
    dependency_cache = work / "cache"
    dependency_cache.mkdir()
    environment = dict(environment, SROBOTIS_THIRDPARTY_CACHE=str(dependency_cache))
    environment.pop("GL4ES_ROOT", None)
    source = work / "consumer"
    source.mkdir()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(gl4es_dependency_contract LANGUAGES C)\n"
        f'include("{modules}/FindGL4ES.cmake")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/resolved.txt" '
        '"${GL4ES_ROOT}\\n${GL4ES_INCLUDE_DIR}\\n${GL4ES_LIB}\\n")\n'
    )

    def configure_gl4es(build, *options, expected=True, env=None):
        result = subprocess.run(
            ["cmake", "-S", str(source), "-B", str(build),
             "-DSROBOTIS_THIRDPARTY_FETCH_OFF=ON", *options],
            cwd=work, env=env or environment, stdin=subprocess.DEVNULL,
            capture_output=True, text=True, timeout=60, check=False,
        )
        output = result.stdout + result.stderr
        assert (result.returncode == 0) == expected, output
        return output

    def seed(root):
        (root / "src/gl").mkdir(parents=True)
        (root / "include/GL").mkdir(parents=True)
        (root / "include/GL/gl.h").write_text("/* dependency test header */\n")
        (root / "dummy.c").write_text("int gl4es_fixture(void) { return 1; }\n")
        (root / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.16)\nproject(gl4es_fixture LANGUAGES C)\n"
            "add_library(GL SHARED dummy.c)\n"
            'set_target_properties(GL PROPERTIES SUFFIX ".so.1" '
            'LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")\n'
        )
        for patch in modules.glob("gl4es-*.patch"):
            for change in patch.read_text().split("diff --git ")[1:]:
                lines = change.splitlines()
                path = root / next(line[6:] for line in lines if line.startswith("--- a/"))
                # Minimal upstream contexts let git apply exercise each real patch.
                original = [line[1:] for line in lines
                            if line.startswith((" ", "-")) and not line.startswith("---")]
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("\n".join(original) + "\n")

    old = dependency_cache / "gl4es"
    seed(old)
    (old / "lib").mkdir()
    (old / "lib/libGL.so").write_text("old shared humanoid library\n")
    old_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in old.rglob("*") if path.is_file()}
    build = work / "managed-build"
    output = configure_gl4es(build, expected=False)
    name = re.search(r"manipulation-gl4es-[0-9a-f]{64}", output).group()
    managed = dependency_cache / name
    assert str(managed) in " ".join(output.split()), output
    seed(managed)
    stencil = modules / "gl4es-imgtec-stencil.patch"
    mipmap = modules / "gl4es-empty-texture-mipmap.patch"
    patches = tuple(modules.glob("gl4es-*.patch"))
    run(["git", "apply", stencil], managed, environment)
    configure_gl4es(
        build, f"-D_g4_inc:PATH={old / 'include'}",
        f"-D_g4_lib:FILEPATH={old / 'lib/libGL.so'}",
        f"-DGL4ES_INCLUDE_DIR:PATH={old / 'include'}",
        f"-DGL4ES_LIB:FILEPATH={old / 'lib/libGL.so'}",
    )
    resolved = (build / "resolved.txt").read_text().splitlines()
    assert Path(resolved[0]) == managed and Path(resolved[2]).parent == managed / "lib"
    for patch in patches:
        run(["git", "apply", "--reverse", "--check", patch], managed, environment)
    library = managed / "lib/libGL.so.1"
    receipt = managed / ".manipulation-gl4es-build"
    digest = hashlib.sha256(library.read_bytes()).hexdigest()
    assert receipt.read_text().splitlines()[1] == digest
    unchanged = (receipt.stat().st_mtime_ns, library.stat().st_mtime_ns)
    configure_gl4es(build)
    assert unchanged == (receipt.stat().st_mtime_ns, library.stat().st_mtime_ns)

    # A newer, replaced output must be rebuilt, never merely given a new receipt.
    library.write_text("unverified replacement\n")
    timestamp = library.stat().st_mtime + 3600
    os.utime(library, (timestamp, timestamp))
    configure_gl4es(build)
    assert hashlib.sha256(library.read_bytes()).hexdigest() == digest
    assert receipt.read_text().splitlines()[1] == digest

    relocated = work / "relocated dependency"
    shutil.copytree(managed, relocated, symlinks=True, ignore=shutil.ignore_patterns("build"))
    explicit = work / "explicit-build"
    configure_gl4es(explicit, f"-DGL4ES_ROOT={relocated}", env=dict(environment, GL4ES_ROOT=str(old)))
    assert Path((explicit / "resolved.txt").read_text().splitlines()[0]) == relocated
    configure_gl4es(work / "environment-build", env=dict(environment, GL4ES_ROOT=str(relocated)))
    rejected = configure_gl4es(explicit, f"-DGL4ES_ROOT={old}", expected=False)
    assert "cmake -U GL4ES_ROOT" in rejected and "was not modified" in rejected
    # Removing the cached override restores managed selection in the same build.
    configure_gl4es(explicit, "-U", "GL4ES_ROOT")
    assert Path((explicit / "resolved.txt").read_text().splitlines()[0]) == managed
    relocated_receipt = relocated / ".manipulation-gl4es-build"
    accepted_receipt = relocated_receipt.read_text()
    relocated_receipt.write_text("0" * 64 + "\n" + digest + "\n")
    configure_gl4es(explicit, f"-DGL4ES_ROOT={relocated}", expected=False)
    relocated_receipt.write_text(accepted_receipt)
    (relocated / "lib/libGL.so.1").write_text("changed after successful build\n")
    configure_gl4es(explicit, f"-DGL4ES_ROOT={relocated}", expected=False)
    assert (relocated / "lib/libGL.so.1").read_text() == "changed after successful build\n"

    # A changed patch fingerprint selects a different managed directory.
    mipmap.write_text(mipmap.read_text() + "\n")
    output = configure_gl4es(build, expected=False)
    replacement = dependency_cache / re.search(r"manipulation-gl4es-[0-9a-f]{64}", output).group()
    assert replacement != managed
    seed(replacement)
    run(["git", "apply", mipmap], replacement, environment)
    (replacement / "dummy.c").write_text("invalid C source\n")
    configure_gl4es(build, expected=False)
    assert not (replacement / ".manipulation-gl4es-build").exists()
    (replacement / "dummy.c").write_text("int gl4es_fixture(void) { return 1; }\n")
    configure_gl4es(build)
    for patch in patches:
        run(["git", "apply", "--reverse", "--check", patch], replacement, environment)
    assert old_hashes == {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in old.rglob("*") if path.is_file()}
    print("gl4es build: isolated cache, patches, receipts, overrides and rebuilds passed")


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
        check_gl4es(work, environment)
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
