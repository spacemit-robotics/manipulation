# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Read-only historical data comparison. Never promotes calibration to a live device."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_root")
    parser.add_argument("dataset")
    parser.add_argument("solver")
    parser.add_argument("output")
    args = parser.parse_args()
    import numpy as np
    source = Path(args.source_root).resolve()
    spec = importlib.util.spec_from_file_location("legacy_calibration", source / "calibration/core.py")
    legacy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(legacy)
    samples = [json.loads(p.read_text()) for p in sorted(Path(args.dataset).glob("sample_*.json"))]
    dataset = dict(schema_version=1, setup="eye_to_hand", samples=[dict(
        T_base_flange=np.asarray(s["T_base_flange"]).reshape(-1).tolist(),
        T_camera_target=np.asarray(s["T_camera_board"]).reshape(-1).tolist()) for s in samples])
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    filename = output / "dataset.json"
    filename.write_text(json.dumps(dataset))
    result = subprocess.run([args.solver, "solve", str(filename)], capture_output=True, text=True, check=False)
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr)
    report = json.loads(result.stdout)
    train = [s for i, s in enumerate(samples) if i % 4 != 0]
    original = legacy.fit([legacy.matrix(s["T_base_flange"]) for s in train],
                          [legacy.matrix(s["T_camera_board"]) for s in train])
    error = float(np.max(np.abs(np.asarray(report["T_base_camera"]).reshape(4, 4) - original)))
    if error > 1e-7:
        raise RuntimeError("C++/legacy fit discrepancy: " + str(error))
    report.update(sample_count=len(samples), legacy_same_training_max_entry_error=error,
                  historical_comparison_only=True)
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
