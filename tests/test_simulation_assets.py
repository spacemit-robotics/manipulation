#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline regression tests for simulation assets and relocatable model packaging."""

import hashlib
import importlib.util
import io
import json
import shutil
import tempfile
import unittest
import urllib.error
from pathlib import Path
from xml.etree import ElementTree


COMPONENT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = COMPONENT_ROOT / "tools/prepare_simulation_assets.py"
SPEC = importlib.util.spec_from_file_location("prepare_simulation_assets", SCRIPT)
ASSETS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ASSETS)


def make_manifest(content=b"mesh"):
    """Return a small pinned manifest that never requires a public server."""
    return {
        "schema_version": 1,
        "source_url": "https://example.invalid/pinned-revision",
        "files": {"robot/assets/mesh.obj": hashlib.sha256(content).hexdigest()},
    }


class SimulationAssetsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "output"
        self.manifest = make_manifest()
        self.calls = []

    def opener(self, url, timeout):
        self.calls.append((url, timeout))
        return io.BytesIO(b"mesh")

    def test_atomic_download_and_idempotence(self):
        self.assertEqual(ASSETS.prepare_assets(self.manifest, self.output, opener=self.opener), 1)
        target = self.output / "robot/assets/mesh.obj"
        self.assertEqual(target.read_bytes(), b"mesh")
        self.assertEqual(list(target.parent.glob("*.part")), [])
        self.assertEqual(ASSETS.prepare_assets(self.manifest, self.output, opener=self.opener), 0)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(
            self.calls[0][0], "https://example.invalid/pinned-revision/robot/assets/mesh.obj"
        )

    def test_check_does_not_create_or_download(self):
        with self.assertRaisesRegex(RuntimeError, "missing or invalid"):
            ASSETS.prepare_assets(self.manifest, self.output, check_only=True, opener=self.opener)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.calls, [])

    def test_check_success_uses_no_transport(self):
        ASSETS.prepare_assets(self.manifest, self.output, opener=self.opener)
        self.calls.clear()
        self.assertEqual(
            ASSETS.prepare_assets(self.manifest, self.output, check_only=True, opener=self.opener),
            0,
        )
        self.assertEqual(self.calls, [])

    def test_mismatch_preserves_existing_file(self):
        target = self.output / "robot/assets/mesh.obj"
        target.parent.mkdir(parents=True)
        target.write_bytes(b"previous data")
        invalid = make_manifest(b"different data")
        with self.assertRaisesRegex(RuntimeError, "SHA-256 mismatch"):
            ASSETS.prepare_assets(invalid, self.output, opener=self.opener)
        self.assertEqual(target.read_bytes(), b"previous data")
        self.assertEqual(list(target.parent.glob("*.part")), [])

    def test_transport_failure_preserves_existing_file(self):
        target = self.output / "robot/assets/mesh.obj"
        target.parent.mkdir(parents=True)
        target.write_bytes(b"previous data")

        def unavailable(_url, timeout):
            self.assertEqual(timeout, 60)
            raise urllib.error.URLError("offline")

        with self.assertRaises(urllib.error.URLError):
            ASSETS.prepare_assets(self.manifest, self.output, opener=unavailable)
        self.assertEqual(target.read_bytes(), b"previous data")
        self.assertEqual(list(target.parent.glob("*.part")), [])

    def test_local_mirror_preserves_pinned_hashes(self):
        mirror = self.root / "mirror"
        source = mirror / "robot/assets/mesh.obj"
        source.parent.mkdir(parents=True)
        source.write_bytes(b"mesh")
        self.assertEqual(
            ASSETS.prepare_assets(self.manifest, self.output, base_url=mirror.as_uri()), 1
        )
        self.assertEqual((self.output / "robot/assets/mesh.obj").read_bytes(), b"mesh")

    def test_invalid_paths_rejected_before_writes(self):
        for relative_path in (
            ".",
            "../outside.obj",
            "robot/../../outside.obj",
            "/absolute.obj",
            "robot//mesh.obj",
            "robot/./mesh.obj",
            "robot\\mesh.obj",
            "robot/%2e%2e/mesh.obj",
            "https://example.invalid/mesh.obj",
        ):
            with self.subTest(relative_path=relative_path):
                manifest = make_manifest()
                manifest["files"][relative_path] = hashlib.sha256(b"mesh").hexdigest()
                with self.assertRaisesRegex(ValueError, "unsafe asset path"):
                    ASSETS.prepare_assets(manifest, self.output, opener=self.opener)
                self.assertFalse(self.output.exists())
        self.assertEqual(self.calls, [])

    def test_symlink_escape_rejected(self):
        self.output.mkdir()
        (self.output / "robot").symlink_to(self.root, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "escapes output"):
            ASSETS.prepare_assets(self.manifest, self.output, opener=self.opener)
        self.assertEqual(self.calls, [])

    def test_invalid_manifest_rejected(self):
        manifest_path = self.root / "assets.json"
        for key, value in (
            ("schema_version", 2),
            ("schema_version", True),
            ("source_url", "http://example.invalid"),
            ("files", {}),
            ("files", {"robot/assets/mesh.obj": "invalid"}),
        ):
            manifest = make_manifest()
            manifest[key] = value
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                ASSETS.load_manifest(manifest_path)

    def test_manifest_and_script_relocate_together(self):
        prefix = self.root / "relocated/share/manipulation"
        script_copy = prefix / "tools/prepare_simulation_assets.py"
        script_copy.parent.mkdir(parents=True)
        shutil.copy2(SCRIPT, script_copy)
        spec = importlib.util.spec_from_file_location("relocated_assets", script_copy)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(
            module.DEFAULT_MANIFEST,
            prefix / "descriptions/simulation/ur5e_robotiq/assets.json",
        )

    def test_bundled_manifest_contains_only_pinned_robot_meshes(self):
        manifest = ASSETS.load_manifest(ASSETS.DEFAULT_MANIFEST)
        self.assertEqual(len(manifest["files"]), 28)
        self.assertTrue(manifest["source_url"].endswith("da76818e269b82289eba39808e2fb91d679d6994"))
        self.assertEqual(
            {Path(path).parts[0] for path in manifest["files"]},
            {"universal_robots_ur5e", "robotiq_2f85"},
        )
        model_root = ASSETS.DEFAULT_MANIFEST.parent / "models"
        for folder in ("universal_robots_ur5e", "robotiq_2f85"):
            self.assertTrue((model_root / folder / "LICENSE").is_file())

    def test_scene_has_no_application_grasp_weld(self):
        root = ASSETS.DEFAULT_MANIFEST.parent
        for model in root.rglob("*.xml"):
            tree = ElementTree.parse(model)
            self.assertEqual(tree.findall(".//weld"), [], model)
        scene = ElementTree.parse(root / "scene.xml")
        self.assertIsNotNone(scene.find(".//geom[@name='floor']"))


if __name__ == "__main__":
    unittest.main()
