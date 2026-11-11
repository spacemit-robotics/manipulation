#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Fault-injecting loopback peer; runs in a remote-only build without any MuJoCo library."""

import json
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLI = os.environ["MANIPULATION_DEVICE"]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402


def receive(connection):
    def exact(count):
        data = b""
        while len(data) < count:
            part = connection.recv(count - len(data))
            if not part:
                raise EOFError
            data += part
        return data

    size = struct.unpack("!I", exact(4))[0]
    if not 0 < size <= 65536:
        raise ValueError("oversized request")
    return json.loads(exact(size))


def send(connection, response):
    payload = json.dumps(response, separators=(",", ":")).encode()
    connection.sendall(struct.pack("!I", len(payload)) + payload)


class FakeServer:
    def __init__(self, mode):
        self.mode = mode
        self.epoch = "a" * 32
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.stopped = threading.Event()
        self.connections = []
        self.workers = []
        self.operations = []
        self.sample = 0
        self.command = 0
        self.target = [0, 0]
        self.thread = threading.Thread(target=self.accept, daemon=True)
        self.thread.start()

    def accept(self):
        while not self.stopped.is_set():
            try:
                connection, _ = self.listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            connection.settimeout(3)
            self.connections.append(connection)
            worker = threading.Thread(target=self.serve, args=(connection,), daemon=True)
            self.workers.append(worker)
            worker.start()

    def state(self):
        if self.mode != "cached" or not self.sample:
            self.sample += 1
        sequence = 0 if self.mode == "old_watermark" else self.command
        return dict(
            arm=dict(
                sample_id=self.sample,
                age_s=1 if self.mode == "stale" else (-0.000000001 if self.mode == "negative_age" else 0),
                sim_time=self.sample * 0.002,
                positions=self.target,
                moving=False,
                active=False,
                command_seq=sequence,
            ),
            effector=None,
        )

    def serve(self, connection):
        try:
            while not self.stopped.is_set():
                request = receive(connection)
                operation = request["op"]
                self.operations.append(operation)
                data = {}
                if operation == "hello":
                    if self.mode == "oversized":
                        connection.sendall(struct.pack("!I", 65537))
                        return
                    if self.mode == "half_header":
                        connection.sendall(b"\x00\x00")
                        self.stopped.wait(3)
                        return
                    if request["role"] == "controller":
                        data = dict(token="b" * 32, fence=1, lease_ms=1000)
                elif operation == "state":
                    if self.mode == "delayed":
                        self.stopped.wait(0.25)
                    data = self.state()
                elif operation == "arm_point":
                    self.command = request["id"]
                    self.target = request["target"]
                    if self.mode == "lost_ack":
                        return
                    data = dict(command_seq=self.command - (self.mode == "bad_ack"))
                elif operation == "arm_hold":
                    data = dict(command_seq=request["id"], stopped=True)
                epoch = "c" * 32 if self.mode == "epoch" and operation == "state" else self.epoch
                send(connection, dict(v=1, id=request["id"], ok=True, epoch=epoch, data=data))
        except (EOFError, OSError):
            pass
        finally:
            connection.close()

    def close(self):
        self.stopped.set()
        self.listener.close()
        self.thread.join(timeout=2)
        for connection in self.connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        for worker in self.workers:
            worker.join(timeout=2)


class TransportTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="mujoco-transport-contract-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        shutil.copyfile(ROOT / "tests/fixtures/mujoco/mapping.yaml", self.directory / "mapping.yaml")
        self.filename = self.directory / "remote.json"

    def peer(self, mode, maximum_age_s=None):
        server = FakeServer(mode)
        self.addCleanup(server.close)
        plan = dict(
            schema_version=3,
            device=dict(
                arm=dict(
                    driver="mujoco_mapped_arm",
                    device_id="arm",
                    resource_id=f"transport_{os.getpid()}_{server.port}",
                    joints=[
                        dict(name="second_joint", minimum_rad=-1, maximum_rad=1),
                        dict(name="first_joint", minimum_rad=-1, maximum_rad=1),
                    ],
                    options=dict(mapping_file="mapping.yaml", transport="tcp", port=str(server.port)),
                ),
            ),
            steps=[],
        )
        if maximum_age_s is not None:
            plan["device"]["arm"]["maximum_age_s"] = maximum_age_s
        self.filename.write_text(json.dumps(plan), encoding="utf-8")
        return server

    def test_epoch_stale_rtt_oversized_and_half_header_fail_closed(self):
        for mode in ("epoch", "stale", "negative_age", "delayed", "oversized", "half_header"):
            with self.subTest(mode=mode):
                peer = self.peer(mode)
                started = time.monotonic()
                result = subprocess.run([CLI, "inspect", str(self.filename)], capture_output=True, text=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertLess(time.monotonic() - started, 4)
                self.assertNotIn("arm_point", peer.operations)
                peer.close()

    def test_client_budget_still_bounds_transport_and_source_age(self):
        for mode, budget, succeeds in (
            ("delayed", 0.05, False),
            ("delayed", 1, True),
            ("stale", 1, False),
        ):
            with self.subTest(mode=mode, maximum_age_s=budget):
                peer = self.peer(mode, maximum_age_s=budget)
                result = subprocess.run([CLI, "inspect", str(self.filename)], capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode == 0, succeeds, result.stderr)
                if not succeeds:
                    self.assertIn("stale/regressed feedback", result.stderr)
                self.assertNotIn("arm_point", peer.operations)
                peer.close()

    def test_old_command_snapshot_cannot_prove_arrival(self):
        self.peer("old_watermark")
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError) as error:
                client.move_arm([0.1, -0.1], speed_scale=0.8)
            self.assertNotEqual(error.exception.result["status"], "succeeded")

    def test_cached_sample_is_not_retimestamped_by_read(self):
        self.peer("cached")
        with DeviceClient(CLI, self.filename) as client:
            client.state()
            time.sleep(0.25)
            with self.assertRaises(DeviceCommandError):
                client.state()

    def test_lost_acceptance_is_not_replayed_or_reported_stopped(self):
        peer = self.peer("lost_ack")
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError) as error:
                client.move_arm([0.1, -0.1], speed_scale=0.8)
            self.assertEqual(error.exception.result["status"], "stop_unknown")
        self.assertEqual(peer.operations.count("arm_point"), 1)

    def test_wrong_acceptance_watermark_latches_connection(self):
        peer = self.peer("bad_ack")
        with DeviceClient(CLI, self.filename) as client:
            with self.assertRaises(DeviceCommandError) as error:
                client.move_arm([0.1, -0.1], speed_scale=0.8)
            self.assertEqual(error.exception.result["status"], "stop_unknown")
        self.assertEqual(peer.operations.count("arm_point"), 1)


if __name__ == "__main__":
    unittest.main()
