#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Bounded read-only camera protocol tests; no physics or display required."""

import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import unittest


CAMERA = os.environ.get("MANIPULATION_SIMULATION_CAMERA")


def receive(connection):
    def exact(size):
        result = b""
        while len(result) < size:
            chunk = connection.recv(size - len(result))
            if not chunk:
                raise EOFError("client closed")
            result += chunk
        return result
    length, = struct.unpack("!I", exact(4))
    if length > 65536:
        raise ValueError("oversized request")
    return json.loads(exact(length))


def send(connection, value, split=False):
    payload = value if isinstance(value, bytes) else json.dumps(value).encode()
    frame = struct.pack("!I", len(payload)) + payload
    if split:
        for offset in range(0, len(frame), 3):
            connection.sendall(frame[offset:offset + 3])
    else:
        connection.sendall(frame)


@unittest.skipUnless(CAMERA, "MANIPULATION_SIMULATION_CAMERA is required")
class CameraTest(unittest.TestCase):
    def exercise(self, fault=None, split=False):
        with socket.socket() as listener, tempfile.TemporaryDirectory() as temporary:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(5)
            port = listener.getsockname()[1]
            errors = []

            def server():
                try:
                    with listener.accept()[0] as connection:
                        connection.settimeout(5)
                        greeting = receive(connection)
                        self.assertEqual(greeting["role"], "observer")
                        send(connection, {"v": 1, "id": 1, "ok": True, "epoch": "epoch-a",
                                          "data": {"role": "observer"}}, split)
                        request = receive(connection)
                        self.assertEqual(request["op"], "capture")
                        data = {"camera": "overview", "width": 2, "height": 1,
                                "sample_id": 42, "sim_time": 0.084,
                                "K": [1, 0, 0.5, 0, 1, 0, 0, 0, 1],
                                "T_world_camera": [1, 0, 0, 0, 0, 1, 0, 0,
                                                   0, 0, 1, 0, 0, 0, 0, 1],
                                "rgb_bytes": 6, "depth_bytes": 8}
                        response = {"v": 1, "id": 2, "ok": True, "epoch": "epoch-a",
                                    "data": data}
                        image = bytes([255, 0, 0, 0, 255, 0]) + struct.pack("<ff", 1.25, 0)
                        if fault == "epoch":
                            response["epoch"] = "epoch-b"
                        elif fault == "dimension":
                            data["width"] = 3
                        elif fault == "intrinsics":
                            data["K"] = [1]
                        elif fault == "depth":
                            image = image[:6] + struct.pack("<ff", float("nan"), 0)
                        send(connection, response, split)
                        if fault in {"epoch", "dimension", "intrinsics"}:
                            return
                        if fault == "oversize":
                            connection.sendall(struct.pack("!I", 33 * 1024 * 1024))
                        elif fault == "partial":
                            connection.sendall(struct.pack("!I", len(image)) + image[:3])
                        else:
                            send(connection, image, split)
                except Exception as error:
                    errors.append(error)

            thread = threading.Thread(target=server, daemon=True)
            thread.start()
            output = Path(temporary) / "frame"
            result = subprocess.run(
                [CAMERA, "--output", str(output), "--port", str(port),
                 "--width", "2", "--height", "1"],
                text=True, capture_output=True, timeout=12)
            thread.join(6)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors, errors)
            if fault:
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertFalse(output.exists(), "invalid frames must not create output")
            else:
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual((output / "rgb.ppm").read_bytes(),
                                 b"P6\n2 1\n255\n" + bytes([255, 0, 0, 0, 255, 0]))
                self.assertEqual(struct.unpack("<ff", (output / "depth.f32").read_bytes()), (1.25, 0))
                metadata = json.loads((output / "metadata.json").read_text())
                self.assertEqual(metadata["epoch"], "epoch-a")
                self.assertEqual(metadata["data"]["sample_id"], 42)

    def test_split_packets(self):
        self.exercise(split=True)

    def test_oversize_frame(self):
        self.exercise("oversize")

    def test_partial_frame_eof(self):
        self.exercise("partial")

    def test_server_epoch_changed(self):
        self.exercise("epoch")

    def test_dimension_mismatch(self):
        self.exercise("dimension")

    def test_invalid_intrinsics(self):
        self.exercise("intrinsics")

    def test_nonfinite_depth(self):
        self.exercise("depth")

    def test_existing_output_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run([CAMERA, "--output", temporary], capture_output=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b"already exists", result.stderr)


if __name__ == "__main__":
    unittest.main()
