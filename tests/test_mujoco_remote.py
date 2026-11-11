#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Loopback transport, writer fencing and independent watchdog over a real headless world."""

import copy
import json
import os
from pathlib import Path
import select
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLI = os.environ["MANIPULATION_DEVICE"]
SERVER = os.environ["MANIPULATION_SIMULATION"]
sys.path.insert(0, str(ROOT / "bindings/python"))
from spacemit_manipulation import DeviceClient, DeviceCommandError  # noqa: E402


def frame(value):
    payload = json.dumps(value, separators=(",", ":")).encode()
    return struct.pack("!I", len(payload)) + payload


def read_exact(connection, count):
    data = b""
    while len(data) < count:
        part = connection.recv(count - len(data))
        if not part:
            raise EOFError("incomplete simulation frame")
        data += part
    return data


def receive(connection):
    size = struct.unpack("!I", read_exact(connection, 4))[0]
    if not 0 < size <= 65536:
        raise ValueError("oversized simulation reply")
    return json.loads(read_exact(connection, size))


class Peer:
    def __init__(self, port, role="observer", contract=None):
        self.connection = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.identifier = 1
        hello = dict(v=1, id=1, op="hello", role=role)
        if contract is not None:
            hello["contract"] = contract
        self.connection.sendall(frame(hello))
        reply = receive(self.connection)
        if not reply["ok"]:
            self.connection.close()
            raise RuntimeError(reply["error"])
        self.epoch = reply["epoch"]
        self.lease = reply["data"] if role == "controller" else {}

    def request(self, operation, **fields):
        self.identifier += 1
        request = dict(v=1, id=self.identifier, op=operation, epoch=self.epoch, **fields)
        if self.lease:
            request.update(token=self.lease["token"], fence=self.lease["fence"])
        return request

    def call(self, operation, **fields):
        request = self.request(operation, **fields)
        self.connection.sendall(frame(request))
        reply = receive(self.connection)
        if reply["id"] != request["id"] or reply["epoch"] != self.epoch:
            raise RuntimeError("response identity mismatch")
        return reply

    def close(self):
        self.connection.close()


class MujocoRemoteTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="mujoco-remote-contract-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        for name in ("scene.xml", "mapping.yaml"):
            shutil.copyfile(ROOT / "tests/fixtures/mujoco" / name, self.directory / name)
        self.plan = dict(
            schema_version=3,
            device=dict(
                arm=dict(
                    driver="mujoco",
                    device_id="arm",
                    resource_id="remote_contract",
                    joints=[
                        dict(name="second_joint", minimum_rad=-1, maximum_rad=1),
                        dict(name="first_joint", minimum_rad=-1, maximum_rad=1),
                    ],
                    timeout_s=5,
                    tolerance_rad=0.005,
                    options=dict(mapping_file="mapping.yaml"),
                ),
                effector=dict(
                    driver="mujoco",
                    device_id="tool",
                    resource_id="remote_contract",
                    channels=["gripper"],
                    timeout_s=5,
                    tolerance=0.01,
                    options=dict(mapping_file="mapping.yaml"),
                ),
            ),
            steps=[],
        )
        self.local_path = self.directory / "local.json"
        self.remote_path = self.directory / "remote.json"
        self.local_path.write_text(json.dumps(self.plan), encoding="utf-8")
        # gl4es can print a library-constructor banner before the server's JSON readiness line.
        # Suppress only that banner; retain stderr and all server/protocol errors.
        self.environment = dict(
            os.environ,
            MANIPULATION_LOCK_DIRECTORY=str(self.directory),
            LIBGL_NOBANNER="1",
        )
        self.server = None
        self.addCleanup(self.stop_server)
        self.start_server()

    def start_server(self):
        self.server = subprocess.Popen(
            [SERVER, "--plan", str(self.local_path), "--port", "0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=self.environment,
        )
        readable, _, _ = select.select([self.server.stdout], [], [], 8)
        self.assertTrue(readable, "simulation server did not become ready")
        line = self.server.stdout.readline()
        self.assertTrue(line, self.server.stderr.read() if self.server.poll() is not None else "")
        ready = json.loads(line)
        self.port = ready["port"]
        self.epoch = ready["epoch"]
        self.remote = copy.deepcopy(self.plan)
        for role in self.remote["device"].values():
            role["options"].update(transport="tcp", host="127.0.0.1", port=str(self.port))
        self.save()

    def stop_server(self):
        if self.server is not None:
            if self.server.poll() is None:
                self.server.terminate()
            try:
                self.server.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                self.server.kill()
                self.server.communicate(timeout=3)
                self.fail("simulation server did not stop within its hold bound")
            self.server = None

    def save(self):
        self.remote_path.write_text(json.dumps(self.remote), encoding="utf-8")

    def peer(self, role="observer"):
        contract = None
        if role == "controller":
            with_observer = Peer(self.port)
            try:
                contract = with_observer.call("describe")["data"]["contract"]
            finally:
                with_observer.close()
        peer = Peer(self.port, role, contract)
        self.addCleanup(peer.close)
        return peer

    def state(self):
        observer = self.peer()
        try:
            return observer.call("state")["data"]
        finally:
            observer.close()

    def wait_still(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            state = self.state()
            if not state["arm"]["moving"] and not state["arm"]["active"]:
                return state
            time.sleep(0.03)
        self.fail("server did not independently hold after controller loss")

    def test_public_motion_and_small_slow_effector_keep_real_feedback(self):
        with DeviceClient(CLI, self.remote_path) as client:
            self.assertTrue(client.capabilities()["arm"]["joint_point"])
            self.assertEqual(client.move_arm([0.12, -0.08], speed_scale=0.8)["status"], "succeeded")
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0.12, delta=0.005)
            before = client.state()["effector_positions"][0]
            started = time.monotonic()
            self.assertEqual(client.move_effector([before + 0.001], duration_s=0.4)["status"], "succeeded")
            self.assertGreaterEqual(time.monotonic() - started, 0.38)
            with self.assertRaises(DeviceCommandError):
                client.enable_arm()

    def test_idle_heartbeat_and_reconnect_do_not_reset_world(self):
        with DeviceClient(CLI, self.remote_path) as client:
            client.move_arm([0.1, -0.1], speed_scale=1)
            time.sleep(5.2)
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0.1, delta=0.005)
        self.wait_still()
        time.sleep(0.1)
        with DeviceClient(CLI, self.remote_path) as client:
            self.assertAlmostEqual(client.state()["joints_rad"][0], 0.1, delta=0.005)

    def test_client_network_age_budget_can_differ_from_server_sampling_budget(self):
        # The server keeps its default 0.2 s sampling budget. These are receiver-side network
        # budgets, so neither may change the physical mapping/limits/timeout contract.
        self.remote["device"]["arm"]["maximum_age_s"] = 1
        self.remote["device"]["effector"]["maximum_age_s"] = 0.7
        self.save()
        observer = self.peer()
        contract = json.loads(observer.call("describe")["data"]["contract"])
        for role in ("arm", "effector"):
            self.assertNotIn("maximum_age", contract[role])
        with DeviceClient(CLI, self.remote_path) as client:
            self.assertEqual(client.move_arm([0.08, -0.05], speed_scale=0.8)["status"], "succeeded")
            self.assertEqual(client.move_effector([0.4], duration_s=0.2)["status"], "succeeded")

    def test_distinct_host_resource_alias_cannot_bypass_server_writer_lease(self):
        with DeviceClient(CLI, self.remote_path) as client:
            for role in self.remote["device"].values():
                role["resource_id"] = "different_local_alias"
            self.save()
            result = subprocess.run(
                [CLI, "inspect", str(self.remote_path)],
                env=self.environment,
                text=True,
                capture_output=True,
                timeout=8,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("lease/contract rejected", result.stderr)
            self.assertTrue(client.state()["enabled"])

    def test_offline_validate_never_connects_or_loads_client_scene(self):
        for role in self.remote["device"].values():
            role["options"]["port"] = "1"
        self.save()
        (self.directory / "scene.xml").unlink()
        for operation in ("validate", "capabilities"):
            result = subprocess.run([CLI, operation, str(self.remote_path)], text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_changed_mapping_contract_rejected_before_motion(self):
        path = self.directory / "mapping.yaml"
        path.write_text(path.read_text().replace("max_velocity_rad_s: 0.6", "max_velocity_rad_s: 0.5"))
        result = subprocess.run(
            [CLI, "inspect", str(self.remote_path)],
            text=True,
            capture_output=True,
            timeout=8,
            env=self.environment,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.state()["arm"]["positions"], [0, 0])

    def test_partial_and_coalesced_frames_are_correlated(self):
        peer = self.peer("controller")
        first = frame(peer.request("state"))
        for byte in first:
            peer.connection.sendall(bytes([byte]))
        self.assertTrue(receive(peer.connection)["ok"])
        second = peer.request("state")
        third = peer.request("state")
        peer.connection.sendall(frame(second) + frame(third))
        self.assertEqual(receive(peer.connection)["id"], second["id"])
        self.assertEqual(receive(peer.connection)["id"], third["id"])

    def test_half_command_eof_never_dispatches_and_holds_previous_target(self):
        peer = self.peer("controller")
        self.assertTrue(peer.call("arm_point", target=[0.8, -0.8], speed_scale=0.2)["ok"])
        time.sleep(0.08)
        data = frame(peer.request("arm_point", target=[-0.8, 0.8], speed_scale=1))
        peer.connection.sendall(data[: len(data) // 2])
        peer.close()
        held = self.wait_still()["arm"]["positions"]
        self.assertGreaterEqual(held[0], -0.002)
        self.assertLess(held[0], 0.3)
        time.sleep(0.12)
        self.assertAlmostEqual(self.state()["arm"]["positions"][0], held[0], delta=0.002)

    def test_watchdog_expiry_fences_old_owner_before_granting_new_one(self):
        old = self.peer("controller")
        old.call("arm_point", target=[0.9, -0.9], speed_scale=0.1)
        time.sleep(1.3)
        held = self.wait_still()["arm"]["positions"]
        self.assertLess(held[0], 0.3)
        new = self.peer("controller")
        self.assertGreater(new.lease["fence"], old.lease["fence"])
        request = new.request("arm_point", target=[-0.9, 0.9], speed_scale=1)
        request.update(token=old.lease["token"], fence=old.lease["fence"])
        new.connection.sendall(frame(request))
        self.assertFalse(receive(new.connection)["ok"])
        time.sleep(0.1)
        self.assertAlmostEqual(self.state()["arm"]["positions"][0], held[0], delta=0.003)

    def assert_disconnect_freezes_both_roles(self, arm_threshold, hold_timeout, timed_out):
        self.stop_server()
        mapping = self.directory / "mapping.yaml"
        mapping.write_text(
            mapping.read_text()
            .replace("arm_moving_threshold_rad_s: 0.004", f"arm_moving_threshold_rad_s: {arm_threshold}")
            .replace("hold_timeout_s: 3", f"hold_timeout_s: {hold_timeout}"),
            encoding="utf-8",
        )
        self.start_server()
        peer = self.peer("controller")
        self.assertTrue(peer.call("arm_point", target=[0.9, -0.9], speed_scale=0.6)["ok"])
        self.assertTrue(peer.call("effector_point", positions=[0.05], duration_s=0.8)["ok"])
        time.sleep(0.12)
        before = peer.call("state")["data"]
        for role in ("arm", "effector"):
            self.assertTrue(before[role]["active"])
            self.assertTrue(before[role]["moving"])
        peer.close()
        started = time.monotonic()
        # State waits behind the revocation's dispatch lock. The final effector position proves
        # its target was frozen before the arm's slow verification, not only after that wait.
        time.sleep(0.06)
        after = self.state()
        self.assertGreater(time.monotonic() - started, 0.2)
        self.assertFalse(after["effector"]["active"])
        self.assertFalse(after["effector"]["moving"])
        self.assertAlmostEqual(after["effector"]["positions"][0], before["effector"]["positions"][0], delta=0.03)
        if timed_out:
            self.assertTrue(after["arm"]["moving"])
            self.assertGreaterEqual(time.monotonic() - started, hold_timeout * 0.9)
            with self.assertRaises(RuntimeError):
                self.peer("controller")
        else:
            self.assertFalse(after["arm"]["moving"])
            next_peer = self.peer("controller")
            self.assertGreater(next_peer.lease["fence"], peer.lease["fence"])

    def test_disconnect_freezes_tool_before_slow_arm_stop_verification(self):
        self.assert_disconnect_freezes_both_roles(1e-8, 3, False)

    def test_arm_hold_timeout_never_defers_effector_target_freeze(self):
        self.assert_disconnect_freezes_both_roles(1e-12, 0.8, True)

    def test_observer_cannot_write_and_does_not_reset_world(self):
        observer = self.peer()
        before = observer.call("state")["data"]["arm"]["sample_id"]
        result = observer.call("arm_point", target=[0.2, 0.2], speed_scale=1)
        self.assertFalse(result["ok"])
        after = observer.call("state")["data"]["arm"]
        self.assertGreaterEqual(after["sample_id"], before)
        self.assertEqual(after["positions"], [0, 0])

    def test_replayed_request_is_not_executed_twice(self):
        peer = self.peer("controller")
        request = peer.request("arm_point", target=[0.5, -0.5], speed_scale=0.2)
        peer.connection.sendall(frame(request))
        self.assertTrue(receive(peer.connection)["ok"])
        peer.connection.sendall(frame(request))
        self.assertFalse(receive(peer.connection)["ok"])
        self.assertLess(self.wait_still()["arm"]["positions"][0], 0.3)

    def test_restart_changes_epoch_and_never_resumes_old_commands(self):
        peer = self.peer("controller")
        peer.call("arm_point", target=[0.8, -0.8], speed_scale=0.2)
        epoch = self.epoch
        self.stop_server()
        self.start_server()
        self.assertNotEqual(self.epoch, epoch)
        self.assertEqual(self.state()["arm"]["positions"], [0, 0])

    def test_non_loopback_endpoint_rejected_offline(self):
        self.remote["device"]["arm"]["options"]["host"] = "192.0.2.10"
        self.save()
        result = subprocess.run([CLI, "validate", str(self.remote_path)], text=True, capture_output=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("loopback", result.stderr)

    def test_server_rejects_non_simulation_plan_before_connecting(self):
        plan = copy.deepcopy(self.plan)
        for role in plan["device"].values():
            role["driver"] = "mock"
            role["options"] = {}
        filename = self.directory / "not_simulation.json"
        filename.write_text(json.dumps(plan), encoding="utf-8")
        result = subprocess.run(
            [SERVER, "--plan", str(filename), "--port", "0"],
            text=True,
            capture_output=True,
            timeout=5,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("only independently selected mujoco", result.stderr)
        self.assertNotIn('"ready"', result.stdout)

    def test_server_command_timeout_does_not_depend_on_client_cancellation(self):
        self.stop_server()
        self.plan["device"]["arm"]["timeout_s"] = 0.12
        self.local_path.write_text(json.dumps(self.plan), encoding="utf-8")
        self.start_server()
        peer = self.peer("controller")
        peer.call("arm_point", target=[0.9, -0.9], speed_scale=0.1)
        time.sleep(0.4)
        self.assertLess(self.wait_still()["arm"]["positions"][0], 0.03)
        next_peer = self.peer("controller")
        self.assertGreater(next_peer.lease["fence"], peer.lease["fence"])

    def test_partial_header_and_oversized_frame_have_bounded_failure(self):
        for payload in (b"\x00\x00", struct.pack("!I", 65537)):
            with self.subTest(payload=payload):
                peer = self.peer("controller")
                peer.connection.sendall(payload)
                self.assertFalse(receive(peer.connection)["ok"])
                peer.close()
                self.wait_still()
                time.sleep(0.05)


if __name__ == "__main__":
    unittest.main()
