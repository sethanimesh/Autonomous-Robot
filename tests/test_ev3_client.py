import json
import unittest

from robot.jetson.ev3_bridge.ev3_client import Ev3Client
from robot.jetson.ev3_bridge.ev3_client import Ev3ConnectionError
from robot.jetson.ev3_bridge.ev3_client import Ev3RemoteError
from robot.jetson.ev3_bridge.ev3_client import parse_args


class FakeClock(object):
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSocket(object):
    def __init__(self, chunk_size=4096, error_command=None):
        self.chunk_size = chunk_size
        self.error_command = error_command
        self.sent = []
        self.pending = b""
        self.closed = False
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, payload):
        request = json.loads(payload.decode("utf-8"))
        self.sent.append(request)
        if request.get("command") == self.error_command:
            response = {"status": "error", "error": "simulated rejection"}
        elif request.get("command") == "ping":
            response = {"status": "ok", "message": "pong", "protocol_version": 1}
        elif request.get("command") == "status":
            response = {"status": "ok", "motion_active": False, "motors": {}}
        elif request.get("command") == "drive":
            response = {"status": "ok", "applied": request}
        elif request.get("command") == "tool_move":
            response = {"status": "ok", "applied": request}
        elif request.get("command") == "tool_home":
            response = {"status": "ok", "applied": request}
        elif request.get("command") == "tool_zero":
            response = {"status": "ok", "position": 0}
        elif request.get("command") == "tool_acknowledge_position":
            response = {"status": "ok", "position": 63}
        else:
            response = {"status": "ok", "stopped": True}
        self.pending += (json.dumps(response) + "\n").encode("utf-8")

    def recv(self, size):
        if not self.pending:
            return b""
        count = min(size, self.chunk_size, len(self.pending))
        data = self.pending[:count]
        self.pending = self.pending[count:]
        return data

    def close(self):
        self.closed = True


class SocketFactory(object):
    def __init__(self, fake_socket):
        self.fake_socket = fake_socket
        self.calls = []

    def __call__(self, address, timeout):
        self.calls.append((address, timeout))
        return self.fake_socket


class Ev3ClientTests(unittest.TestCase):
    def make_client(self, fake_socket=None, clock=None):
        fake_socket = fake_socket or FakeSocket()
        factory = SocketFactory(fake_socket)
        client = Ev3Client(
            socket_factory=factory,
            clock=clock,
            sleeper=clock.sleep if clock else None,
        )
        return client, fake_socket, factory

    def test_ping_connects_to_default_ev3(self):
        client, fake_socket, factory = self.make_client()

        response = client.ping()

        self.assertEqual("pong", response["message"])
        self.assertEqual([(("192.168.1.25", 9999), 2.0)], factory.calls)
        self.assertEqual(2.0, fake_socket.timeout)

    def test_partial_response_chunks_are_reassembled(self):
        client, _, _ = self.make_client(FakeSocket(chunk_size=3))

        response = client.status()

        self.assertFalse(response["motion_active"])

    def test_remote_error_is_reported(self):
        client, _, _ = self.make_client(FakeSocket(error_command="drive"))

        with self.assertRaises(Ev3RemoteError):
            client.drive(10, 10)

    def test_drive_for_refreshes_and_finishes_with_stop(self):
        clock = FakeClock()
        client, fake_socket, _ = self.make_client(clock=clock)

        responses = client.drive_for(80, 80, duration_seconds=0.25)

        commands = [item["command"] for item in fake_socket.sent]
        self.assertEqual(["drive", "drive", "drive", "stop"], commands)
        self.assertTrue(
            all("tool" not in item for item in fake_socket.sent if item["command"] == "drive")
        )
        self.assertEqual(3, len(responses))
        self.assertAlmostEqual(10.25, clock.now)

    def test_drive_for_rejects_unsafe_duration(self):
        client, fake_socket, _ = self.make_client()

        with self.assertRaises(ValueError):
            client.drive_for(80, 80, duration_seconds=5.1)

        self.assertEqual([], fake_socket.sent)

    def test_non_numeric_speed_is_rejected_before_connecting(self):
        client, fake_socket, factory = self.make_client()

        with self.assertRaises(ValueError):
            client.drive("fast", 10)

        self.assertEqual([], fake_socket.sent)
        self.assertEqual([], factory.calls)

    def test_camera_head_position_commands(self):
        client, fake_socket, _ = self.make_client()

        client.move_tool(25, 40)
        client.home_tool()
        client.zero_tool()
        client.acknowledge_tool_position()

        self.assertEqual(
            [
                {"command": "tool_move", "position": 25, "speed": 40},
                {"command": "tool_home", "speed": 300},
                {"command": "tool_zero"},
                {"command": "tool_acknowledge_position"},
            ],
            fake_socket.sent,
        )

    def test_camera_head_cli_uses_the_loaded_home_speed(self):
        args = parse_args(["tool-move", "--position", "25"])

        self.assertEqual("tool-move", args.action)
        self.assertEqual(25, args.position)
        self.assertEqual(40, args.speed)
        self.assertEqual(300, parse_args(["tool-home"]).speed)

    def test_closed_connection_is_reported_and_discarded(self):
        client, fake_socket, _ = self.make_client()
        fake_socket.recv = lambda size: b""

        with self.assertRaises(Ev3ConnectionError):
            client.ping()

        self.assertTrue(fake_socket.closed)
        self.assertIsNone(client.socket)


if __name__ == "__main__":
    unittest.main()
