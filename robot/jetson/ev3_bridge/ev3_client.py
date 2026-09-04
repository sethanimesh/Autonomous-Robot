#!/usr/bin/env python3
"""Small synchronous client for the Echora EV3 JSON service."""

import argparse
import json
import math
import socket
import sys
import time


DEFAULT_EV3_HOST = "192.168.1.25"
DEFAULT_EV3_PORT = 9999
DEFAULT_TIMEOUT_SECONDS = 2.0
DEFAULT_REFRESH_SECONDS = 0.1
MAX_DEVELOPMENT_PULSE_SECONDS = 5.0
MAX_LINE_BYTES = 65536


class Ev3ClientError(Exception):
    """Base error for bridge client failures."""


class Ev3ConnectionError(Ev3ClientError):
    """Raised when the EV3 connection or framing fails."""


class Ev3RemoteError(Ev3ClientError):
    """Raised when the EV3 returns an error response."""


class Ev3Client(object):
    """Sequential request/response client with safe finite-duration driving."""

    def __init__(
        self,
        host=DEFAULT_EV3_HOST,
        port=DEFAULT_EV3_PORT,
        timeout_seconds=DEFAULT_TIMEOUT_SECONDS,
        socket_factory=None,
        clock=None,
        sleeper=None,
    ):
        self.host = host
        self.port = int(port)
        self.timeout_seconds = float(timeout_seconds)
        self.socket_factory = socket_factory or socket.create_connection
        self.clock = clock or time.monotonic
        self.sleeper = sleeper or time.sleep
        self.socket = None
        self.receive_buffer = b""

    def connect(self):
        if self.socket is not None:
            return
        try:
            self.socket = self.socket_factory(
                (self.host, self.port), self.timeout_seconds
            )
            self.socket.settimeout(self.timeout_seconds)
        except (OSError, socket.error) as exc:
            self.socket = None
            raise Ev3ConnectionError("cannot connect to EV3: {0}".format(exc))

    def close(self):
        if self.socket is None:
            return
        try:
            self.socket.close()
        finally:
            self.socket = None
            self.receive_buffer = b""

    def _receive_line(self):
        while b"\n" not in self.receive_buffer:
            try:
                data = self.socket.recv(4096)
            except (OSError, socket.error) as exc:
                raise Ev3ConnectionError("failed to receive response: {0}".format(exc))
            if not data:
                raise Ev3ConnectionError("EV3 closed the connection")
            self.receive_buffer += data
            if len(self.receive_buffer) > MAX_LINE_BYTES:
                raise Ev3ConnectionError("EV3 response line is too long")

        line, self.receive_buffer = self.receive_buffer.split(b"\n", 1)
        return line

    def request(self, command):
        if not isinstance(command, dict):
            raise ValueError("command must be a dictionary")
        self.connect()
        payload = (
            json.dumps(command, separators=(",", ":"), sort_keys=True) + "\n"
        ).encode("utf-8")

        try:
            self.socket.sendall(payload)
            line = self._receive_line()
            response = json.loads(line.decode("utf-8"))
        except Ev3ConnectionError:
            self.close()
            raise
        except (OSError, socket.error, UnicodeError, ValueError) as exc:
            self.close()
            raise Ev3ConnectionError("invalid EV3 response: {0}".format(exc))

        if not isinstance(response, dict):
            self.close()
            raise Ev3ConnectionError("EV3 response is not a JSON object")
        if response.get("status") != "ok":
            raise Ev3RemoteError(response.get("error", "EV3 command failed"))
        return response

    def ping(self):
        return self.request({"command": "ping"})

    def status(self):
        return self.request({"command": "status"})

    def stop(self):
        return self.request({"command": "stop"})

    @staticmethod
    def _validate_speed(name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{0} speed must be a number".format(name))
        if not math.isfinite(value):
            raise ValueError("{0} speed must be finite".format(name))
        return int(round(value))

    def drive(self, left, right, tool=None):
        command = {
            "command": "drive",
            "left": self._validate_speed("left", left),
            "right": self._validate_speed("right", right),
        }
        if tool is not None:
            command["tool"] = self._validate_speed("tool", tool)
        return self.request(command)

    def move_tool(self, position, speed):
        return self.request(
            {
                "command": "tool_move",
                "position": self._validate_speed("tool position", position),
                "speed": self._validate_speed("tool", speed),
            }
        )

    def home_tool(self, speed=25):
        return self.request(
            {
                "command": "tool_home",
                "speed": self._validate_speed("tool home", speed),
            }
        )

    def zero_tool(self):
        return self.request({"command": "tool_zero"})

    def acknowledge_tool_position(self):
        return self.request({"command": "tool_acknowledge_position"})

    def drive_for(
        self,
        left,
        right,
        tool=0,
        duration_seconds=0.25,
        refresh_seconds=DEFAULT_REFRESH_SECONDS,
    ):
        """Refresh a finite drive command and always finish with an explicit stop."""

        duration_seconds = float(duration_seconds)
        refresh_seconds = float(refresh_seconds)
        if duration_seconds <= 0 or duration_seconds > MAX_DEVELOPMENT_PULSE_SECONDS:
            raise ValueError(
                "duration must be greater than zero and no more than {0} seconds".format(
                    MAX_DEVELOPMENT_PULSE_SECONDS
                )
            )
        if refresh_seconds <= 0 or refresh_seconds >= 0.5:
            raise ValueError("refresh interval must be greater than zero and below 0.5s")

        deadline = self.clock() + duration_seconds
        responses = []
        try:
            while self.clock() < deadline:
                responses.append(self.drive(left, right, tool))
                remaining = deadline - self.clock()
                if remaining > 0:
                    self.sleeper(min(refresh_seconds, remaining))
        finally:
            try:
                self.stop()
            except Ev3ClientError:
                # The EV3's independent 500 ms watchdog remains the final fallback.
                pass
        return responses

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self.stop()
        except Ev3ClientError:
            pass
        self.close()
        return False


def print_json(value):
    print(json.dumps(value, indent=2, sort_keys=True))


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_EV3_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_EV3_PORT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    subparsers = parser.add_subparsers(dest="action")
    subparsers.required = True

    subparsers.add_parser("ping")
    subparsers.add_parser("status")
    subparsers.add_parser("stop")
    subparsers.add_parser("tool-zero")
    subparsers.add_parser("tool-acknowledge-position")
    tool_home = subparsers.add_parser("tool-home")
    tool_home.add_argument("--speed", type=int, default=25)

    tool_move = subparsers.add_parser("tool-move")
    tool_move.add_argument("--position", type=int, required=True)
    tool_move.add_argument("--speed", type=int, default=40)

    pulse = subparsers.add_parser("pulse")
    pulse.add_argument("--left", type=int, default=0)
    pulse.add_argument("--right", type=int, default=0)
    pulse.add_argument("--tool", type=int, default=0)
    pulse.add_argument("--duration", type=float, default=0.25)
    pulse.add_argument("--refresh", type=float, default=DEFAULT_REFRESH_SECONDS)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    client = Ev3Client(args.host, args.port, args.timeout)
    try:
        if args.action == "ping":
            print_json(client.ping())
        elif args.action == "status":
            print_json(client.status())
        elif args.action == "stop":
            print_json(client.stop())
        elif args.action == "tool-zero":
            print_json(client.zero_tool())
        elif args.action == "tool-acknowledge-position":
            print_json(client.acknowledge_tool_position())
        elif args.action == "tool-home":
            print_json(client.home_tool(args.speed))
        elif args.action == "tool-move":
            print_json(client.move_tool(args.position, args.speed))
        elif args.action == "pulse":
            responses = client.drive_for(
                args.left,
                args.right,
                args.tool,
                args.duration,
                args.refresh,
            )
            print_json(
                {
                    "refresh_count": len(responses),
                    "final_status": client.status(),
                }
            )
    except (Ev3ClientError, ValueError) as exc:
        print("error: {0}".format(exc), file=sys.stderr)
        return 1
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
