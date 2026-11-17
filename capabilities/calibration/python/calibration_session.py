# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Shared immutable hand-eye sessions, metadata binding and application exports."""

import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .calibration_geometry import matrix, matrix_pose
from .hand_eye import solve, validate_report
from .workbench import load, save_new


def digest(path):
    """Fingerprint a model or configuration without recording a machine-specific path."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity_digest(identity):
    """Stable session identity independent of JSON whitespace and dictionary ordering."""
    return hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def load_config(path):
    """Resolve collection resources relative to the configuration, never the working directory."""
    path = Path(path).resolve()
    config = load(path)
    allowed = {"schema_version", "installation_id", "device_plan", "model", "sampler", "frames"}
    if config.get("schema_version") != 1 or set(config) - allowed:
        raise ValueError("Unsupported calibration configuration")
    if not isinstance(config.get("installation_id"), str) or not config["installation_id"].strip():
        raise ValueError("installation_id must identify the fixed robot/camera installation")
    for field in ("device_plan", "model"):
        config[field] = (path.parent / config[field]).resolve()
        if not config[field].is_file():
            raise FileNotFoundError(str(config[field]))
    sampler = config.get("sampler", {})
    if set(sampler) - {"module", "options", "path_options"}:
        raise ValueError("Unknown sampler configuration field")
    options = dict(sampler.get("options", {}))
    if any(
        not isinstance(key, str) or not isinstance(value, str) for key, value in options.items()
    ):
        raise ValueError("Sampler options must be string pairs")
    path_options = sampler.get("path_options", ["board", "window"])
    if not isinstance(path_options, list) or any(not isinstance(key, str) for key in path_options):
        raise ValueError("sampler.path_options must list file-valued options")
    for key in path_options:
        if key in options:
            # A window rectangle is not a file path.
            if key == "window" and "," in options[key]:
                continue
            options[key] = str((path.parent / options[key]).resolve())
    config["sampler"] = {
        "module": sampler.get("module", "spacemit_manipulation.charuco_sampler"),
        "options": options,
    }
    model = load(config["model"])
    frames = dict(config.get("frames", {}))
    if set(frames) - {"base", "flange", "camera", "target"}:
        raise ValueError("Unknown calibration frame")
    for key, expected in (("base", model["base"]), ("flange", model["tip"])):
        if key in frames and frames[key] != expected:
            raise ValueError(f"{key} frame disagrees with model")
        frames[key] = expected
    frames.setdefault("camera", "color_optical")
    frames.setdefault("target", "calibration_board")
    if any(not isinstance(value, str) or not value.strip() for value in frames.values()):
        raise ValueError("Calibration frames must be nonempty names")
    config["frames"] = frames
    return config


def device_identity(device_plan, model_file, *, installation_id, description=None):
    """Bind a collection to one installation, device configuration and kinematic model."""
    model_file = Path(model_file).resolve()
    model = load(model_file)
    urdf = (model_file.parent / model["urdf"]).resolve()
    if not installation_id or not str(installation_id).strip():
        raise ValueError("installation_id is required")
    return {
        "installation_id": str(installation_id),
        "installation_id_explicit": True,
        "device_plan_sha256": digest(device_plan),
        "model_sha256": digest(model_file),
        "urdf_sha256": digest(urdf),
        "base_frame": model["base"],
        "flange_frame": model["tip"],
        "joints": list(model["joints"]),
        "device": description or {},
    }


def session_identity(device, provenance, frames=None):
    """Bind device, camera calibration, board geometry and coordinate frame names."""
    camera = provenance.get("camera", {})
    board = provenance.get("board", {})
    if not camera.get("serial") or not camera.get("frame") or not board:
        raise ValueError("Calibration requires camera identity, optical frame and board geometry")
    frame_names = frames or {
        "base": device["base_frame"],
        "flange": device["flange_frame"],
        "camera": camera["frame"],
        "target": "calibration_board",
    }
    if (
        frame_names["camera"] != camera["frame"]
        or frame_names["base"] != device["base_frame"]
        or frame_names["flange"] != device["flange_frame"]
    ):
        raise ValueError("Calibration frame names disagree with device or camera")
    return {"device": device, "camera": camera, "board": board, "frames": frame_names}


def read_dataset(path):
    """Read shared or existing Dice sessions; invalid/incomplete samples never enter a solve."""
    path = Path(path)
    if (path / "INVALIDATED.json").exists():
        raise ValueError("Dataset is invalidated; inspect INVALIDATED.json and start a new session")
    if (path / "AUTO_INCOMPLETE.json").exists():
        raise ValueError("Automatic collection is incomplete; inspect AUTO_INCOMPLETE.json")
    if (path / "COLLECTION_INCOMPLETE.json").exists():
        raise ValueError("Collection is incomplete; no complete calibration session is available")
    manifest = load(path / "manifest.json")
    if manifest.get("schema") != 1 or manifest.get("mode") != "eye_to_hand":
        raise ValueError("Unsupported dataset schema/mode")
    tcp = matrix(manifest["T_flange_tcp"])
    sample_paths = sorted(path.glob("sample_*.json"))
    if [sample.name for sample in sample_paths] != [
        f"sample_{index:04d}.json" for index in range(len(sample_paths))
    ]:
        raise ValueError("Session sample numbering has gaps")
    samples = [load(sample) for sample in sample_paths]
    expected_identity = manifest.get("session_identity")
    expected_digest = identity_digest(expected_identity) if expected_identity else None
    if expected_identity and (
        expected_identity.get("camera") != manifest.get("camera")
        or expected_identity.get("board") != manifest.get("board")
    ):
        raise ValueError("Session identity disagrees with camera or board")
    for sample in samples:
        flange = matrix(sample["T_base_flange"])
        matrix(sample["T_camera_board"])
        if "T_base_tcp" in sample and not np.allclose(
            matrix(sample["T_base_tcp"]), flange @ tcp, atol=1e-6, rtol=0
        ):
            raise ValueError("Saved T_base_tcp disagrees with flange and session TCP")
        if expected_digest and sample.get("session_identity_sha256") != expected_digest:
            raise ValueError("Sample belongs to a different calibration session")
        if expected_identity and "joints_rad" in sample:
            joints = np.asarray(sample["joints_rad"], dtype=float)
            if (
                joints.shape != (len(expected_identity["device"]["joints"]),)
                or not np.isfinite(joints).all()
            ):
                raise ValueError("Sample joints disagree with the session model")
    return manifest, samples


class CalibrationSession:
    """Immutable collection storage shared by CLI and application-specific samplers.

    An unfinished collection remains readable as individual evidence files but cannot
    be solved as a complete session. This class never connects to or moves hardware.
    """

    def __init__(self, path, identity, provenance, tcp=None):
        self.path = Path(path).resolve()
        self.identity = json.loads(json.dumps(identity, allow_nan=False))
        self.identity_sha256 = identity_digest(self.identity)
        self.tcp = matrix(np.eye(4) if tcp is None else tcp)
        self.samples = []
        self.path.mkdir(parents=True, exist_ok=False)
        save_new(self.path / "COLLECTION_INCOMPLETE.json", {"created_unix_s": time.time()})
        save_new(
            self.path / "manifest.json",
            {
                "schema": 1,
                "mode": "eye_to_hand",
                "tcp": "flange" if tcp is None else "custom",
                "T_flange_tcp": self.tcp.tolist(),
                "camera": identity["camera"],
                "board": identity["board"],
                "session_identity": identity,
                "sampling": "stationary bracketed observations, "
                "not hardware timestamp synchronization",
                "created_unix_s": time.time(),
            },
        )
        save_new(self.path / "provenance.json", dict(provenance, session_identity=identity))

    def append(self, sample):
        """Validate and publish the next sample. Previously accepted samples are never replaced."""
        sample = dict(sample)
        if "joints_rad" in sample:
            joints = np.asarray(sample["joints_rad"], dtype=float)
            if (
                joints.shape != (len(self.identity["device"]["joints"]),)
                or not np.isfinite(joints).all()
            ):
                raise ValueError("Sample joints disagree with the session model")
        flange = matrix(sample["T_base_flange"])
        matrix(sample["T_camera_board"])
        if "T_base_tcp" in sample and not np.allclose(
            matrix(sample["T_base_tcp"]), flange @ self.tcp, atol=1e-6, rtol=0
        ):
            raise ValueError("Sample TCP disagrees with collection")
        if sample.get("session_identity_sha256", self.identity_sha256) != self.identity_sha256:
            raise ValueError("Sample belongs to a different calibration session")
        sample.update(
            T_base_tcp=(flange @ self.tcp).tolist(), session_identity_sha256=self.identity_sha256
        )
        save_new(self.path / f"sample_{len(self.samples):04d}.json", sample)
        self.samples.append(sample)

    def finish(self):
        """Publish the established flat dataset format as well as the shared session format."""
        if not 12 <= len(self.samples) <= 1000:
            raise ValueError("A complete calibration requires 12..1000 samples")
        save_new(
            self.path / "dataset.json",
            {
                "schema_version": 1,
                "setup": "eye_to_hand",
                "samples": [
                    {
                        "T_base_flange": np.asarray(sample["T_base_flange"]).ravel().tolist(),
                        "T_camera_target": np.asarray(sample["T_camera_board"]).ravel().tolist(),
                    }
                    for sample in self.samples
                ],
            },
        )
        (self.path / "COLLECTION_INCOMPLETE.json").unlink()


def solve_session(path, max_position_m=0.005, max_angle_deg=2.0):
    """Solve a complete session and retain the exact installation and sensor identity."""
    manifest, samples = read_dataset(path)
    report = solve(samples, manifest["T_flange_tcp"], max_position_m, max_angle_deg)
    report.update(
        camera=manifest["camera"],
        board=manifest["board"],
        tcp=manifest.get("tcp", "flange"),
        sample_count=len(samples),
    )
    if "session_identity" in manifest:
        report["session_identity"] = manifest["session_identity"]
    report["source_manifest_sha256"] = digest(Path(path) / "manifest.json")
    return report


def export_perceptive(report, destination, camera_type, installation_id):
    """Write a new perceptive_grasp calibration YAML/JSON fragment, never edit its config.

    Both applications use camera-optical to arm-base transforms in metres and
    XYZ roll/pitch/yaw radians. The operator must name this installation explicitly;
    an extrinsic from another installation is rejected rather than reused.
    """
    validate_report(report)
    identity = report.get("session_identity", {})
    if identity.get("device", {}).get("installation_id") != installation_id:
        raise ValueError("Report does not belong to the requested installation")
    if identity["device"].get("installation_id_explicit") is False:
        raise ValueError("A generated session ID does not identify a confirmed installation")
    if camera_type not in ("realsense", "spacemit_las2"):
        raise ValueError("Unsupported perceptive_grasp camera backend")
    if report["camera"].get("frame") != "color_optical":
        raise ValueError("perceptive_grasp export requires color_optical coordinates")
    if report["camera"].get("backend") not in (None, camera_type):
        raise ValueError("Selected camera backend differs from the calibration camera")
    if report["camera"] != identity.get("camera") or report["board"] != identity.get("board"):
        raise ValueError("Report metadata disagrees with its calibration session")
    pose = matrix_pose(report["T_base_camera"])
    # JSON is a YAML subset, so this fragment can be used by either configuration reader.
    save_new(
        destination,
        {
            "calibration": {
                camera_type: {
                    "T_base_camera": {
                        "translation": pose[:3],
                        "rotation": pose[3:],
                    }
                }
            }
        },
    )
