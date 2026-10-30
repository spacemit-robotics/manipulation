# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline FK, promotion and restore tests; stdlib only, no camera or vendor SDK."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "bindings/python"))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    instance = importlib.util.module_from_spec(spec)
    sys.modules[name] = instance
    spec.loader.exec_module(instance)
    return instance


fk = module("spacemit_manipulation.urdf_fk", "capabilities/kinematics/python/urdf_fk.py")
work = module("spacemit_manipulation.workbench", "capabilities/calibration/cli/workbench.py")


class DependencyTests(unittest.TestCase):
    def test_vendor_dependencies_have_one_metadata_owner(self):
        sdk = ROOT.parents[2]
        vendor = sdk / "components/thirdparty/pyagxarm"
        overlay = sdk / "platform/generic/components/thirdparty/pyagxarm/package.xml"
        self.assertTrue(overlay.is_symlink())
        self.assertTrue(overlay.samefile(vendor / "package.xml"))
        package = ET.parse(ROOT / "package.xml").getroot()
        self.assertIn("pyagxarm", [item.text for item in package.findall("depend")])
        self.assertEqual({item.text for item in package.findall("system_depend")},
                         {"libyaml-cpp-dev", "python3-numpy"})
        dependencies = ET.parse(vendor / "package.xml").getroot().findall("system_depend")
        self.assertEqual({item.text for item in dependencies},
                         {"python3-can", "python3-typing-extensions"})
        self.assertEqual({item.get("check") for item in dependencies},
                         {"python3 -c 'import can'", "python3 -c 'import typing_extensions'"})

    def test_sdk_dependency_closure_preserves_runtime_requirements(self):
        sdk = ROOT.parents[2]
        script = """
set -euo pipefail
REPO_ROOT="$1"
source "$REPO_ROOT/build/nonros2.sh"
while IFS= read -r package; do
    [[ -n "$package" ]] || continue
    printf 'package|%s\\n' "$package"
    read_package_sysdeps_lines "$package"
done < <(resolve_nonros2_dependency_closure "$2" 1)
"""
        for package, expected_packages, expected_dependencies in (
                ("components/thirdparty/pyagxarm", ["components/thirdparty/pyagxarm"],
                 {"python3-can", "python3-typing-extensions"}),
                ("components/control/manipulation",
                 ["components/thirdparty/pyagxarm", "components/control/manipulation"],
                 {"python3-can", "python3-typing-extensions", "libyaml-cpp-dev", "python3-numpy"})):
            with self.subTest(package=package), tempfile.TemporaryDirectory() as temporary:
                environment = dict(os.environ)
                for key in ("BUILD_CONFIG_FILE", "BUILD_TARGET", "BUILD_TARGET_FILE",
                            "SDK_BUILD_ARCH", "SDK_BUILD_BOARD", "SROBOTIS_CROSS_BUILD"):
                    environment.pop(key, None)
                result = subprocess.run(["bash", "-c", script, "dependency-test", str(sdk), package],
                                        cwd=temporary, env=environment, check=True,
                                        capture_output=True, text=True, timeout=15)
                lines = [line.split("|", 2) for line in result.stdout.splitlines()]
                self.assertEqual([line[1] for line in lines if line[0] == "package"],
                                 expected_packages)
                self.assertEqual({line[1] for line in lines if line[0] == "required"},
                                 expected_dependencies)


class ToolTests(unittest.TestCase):
    def test_urdf_order_limits_and_rigidity(self):
        path = ROOT / "descriptions/robots/nero/model.json"
        config = json.loads(path.read_text())
        model = fk.UrdfKinematics(path.parent / config["urdf"], config["base"], config["tip"], config["joints"])
        fk.validate_transform(model.forward([0] * 7))
        with self.assertRaises(ValueError):
            model.forward([100] * 7)
        with self.assertRaises(ValueError):
            fk.validate_transform([0] * 16)
        reflected = fk.identity()
        reflected[0] = -1
        with self.assertRaises(ValueError):
            fk.validate_transform(reflected)

    def test_nero_limits_are_the_vendor_preset(self):
        """Device templates and the URDF carry the pyAgxArm Nero preset unchanged: one source."""
        import ast
        import re
        import xml.etree.ElementTree as ET
        constants = ROOT.parents[1] / "thirdparty/pyagxarm/pyAgxArm/api/constants.py"
        preset = next(ast.literal_eval(node.value)["nero"] for node in ast.walk(ast.parse(constants.read_text()))
                      if isinstance(node, ast.Assign)
                      and any(getattr(t, "id", "") == "ROBOT_JOINT_LIMIT_PRESET_RAD" for t in node.targets))
        urdf = ET.parse(ROOT / "descriptions/robots/nero/nero_description.urdf").getroot()
        self.assertEqual({joint.get("name"): [float(joint.find("limit").get(k)) for k in ("lower", "upper")]
                          for joint in urdf.findall("joint") if joint.get("type") == "revolute"}, preset)
        for template in ("arm.yaml", "revo2_right.yaml"):
            text = (ROOT / "descriptions/robots/nero/devices" / template).read_text()
            found = {name: [float(low), float(high)] for name, low, high in re.findall(
                r"\{name: (joint\d), minimum_rad: ([-0-9.]+), maximum_rad: ([-0-9.]+)\}", text)}
            self.assertEqual(found, preset, template)

    def test_promote_restore_exclusive_create_and_quality_gate(self):
        report = dict(schema_version=1, setup="eye_to_hand", quality_passed=True,
                      T_base_camera=fk.identity(), holdout_translation_m=0, holdout_rotation_rad=0,
                      maximum_translation_m=0, maximum_rotation_rad=0)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            candidate, active, restored = root / "candidate.json", root / "active.json", root / "restored.json"
            work.save_new(candidate, report)
            work.promote(candidate, active)
            self.assertEqual(work.load(active)["T_base_camera"], fk.identity())
            with self.assertRaises(FileExistsError):
                work.promote(candidate, active)
            work.promote(active, restored)
            self.assertEqual(work.load(restored)["T_base_camera"], report["T_base_camera"])
            report["holdout_translation_m"] = 0.01
            with self.assertRaises(ValueError):
                work.validate_report(report)
            report["holdout_translation_m"] = 0
            report["maximum_translation_m"] = 0.01
            with self.assertRaises(ValueError):
                work.validate_report(report)
            report["holdout_translation_m"] = 0
            report["quality_passed"] = False
            with self.assertRaises(ValueError):
                work.validate_report(report)


if __name__ == "__main__":
    unittest.main()
