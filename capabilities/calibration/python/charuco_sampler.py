# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Built-in hand-eye sampler: RealSense colour camera and a ChArUco board on the flange.

Options (--option NAME=VALUE):
  board=FILE            board JSON (required); its image_profile selects resolution, fps and crop
  serial=SERIAL         RealSense serial; required when several cameras are connected
  measured_width_mm=W   printed board width, rescales the board model (with measured_height_mm)
  measured_height_mm=H
  window=FILE|x0,y0,x1,y1  board_window.json or a rectangle the whole board must stay inside
"""
import json
from pathlib import Path

from .charuco import CharucoDetector, RealSenseCamera
from .sampling import capture_sample


def write_image_new(path, image):
    import cv2
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("PNG encoding failed")
    with open(path, "xb") as stream:  # Evidence images are never overwritten.
        stream.write(encoded.tobytes())


def parse_window(value):
    if value is None:
        return None
    path = Path(value)
    if path.is_file():
        window = json.loads(path.read_text(encoding="utf-8"))["window_xyxy"]
    else:
        window = [int(part) for part in value.split(",")]
    if len(window) != 4 or not (0 <= window[0] < window[2] and 0 <= window[1] < window[3]):
        raise ValueError("window must be x0,y0,x1,y1")
    return window


class CharucoSampler:
    def __init__(self, options, camera=None):
        known = {"board", "serial", "measured_width_mm", "measured_height_mm", "window"}
        if set(options) - known or "board" not in options:
            raise ValueError("sampler options: board=FILE (required), serial, measured_width_mm, "
                             "measured_height_mm, window")
        self.board_path = Path(options["board"]).resolve()
        self.board = json.loads(self.board_path.read_text(encoding="utf-8"))
        measured = None
        if "measured_width_mm" in options or "measured_height_mm" in options:
            measured = (float(options["measured_width_mm"]) / 1000, float(options["measured_height_mm"]) / 1000)
        self.detector = CharucoDetector(self.board, measured)
        self.window = parse_window(options.get("window"))
        self.window_source = options.get("window")
        self.camera = camera or RealSenseCamera(options.get("serial"), self.board.get("image_profile"))

    def capture(self, port, prefix):
        record, image, annotated = capture_sample(port, self.camera, self.detector, self.window)
        write_image_new(f"{prefix}.png", image)
        write_image_new(f"{prefix}_detected.png", annotated)
        return record

    def provenance(self):
        return dict(camera=self.camera.info, board=self.detector.cfg, board_file=str(self.board_path),
                    window_xyxy=self.window, sampler="spacemit_manipulation.charuco_sampler")

    def dependencies(self):
        files = [self.board_path]
        if self.window_source and Path(self.window_source).is_file():
            files.append(Path(self.window_source).resolve())
        return files

    def close(self):
        self.camera.close()


def create_sampler(options):
    return CharucoSampler(options)
