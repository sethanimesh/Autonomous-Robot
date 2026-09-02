#!/usr/bin/env python3
"""Small newline-delimited JSON control service for an ev3dev brick.

The module intentionally uses only the Python 3.5 standard library so it can run
on the project's existing ev3dev-stretch installation.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import socket
import sys
import time


PROTOCOL_VERSION = 1
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9999
DEFAULT_ALLOWED_CLIENT = "192.168.1.48"
DEFAULT_WATCHDOG_SECONDS = 0.5
DEFAULT_DRIVE_SPEED_LIMIT = 250
DEFAULT_TOOL_SPEED_LIMIT = 150
DEFAULT_POLL_SECONDS = 0.05
DEFAULT_MAX_LINE_BYTES = 4096

MOTOR_PORTS = {
    "left": "outB",
    "right": "outC",
    "tool": "outA",
}


class ProtocolError(Exception):
    """Raised when a client request is invalid."""


class HardwareError(Exception):
    """Raised when a required EV3 hardware operation fails."""


class SysfsMotor(object):
    """Minimal tacho-motor adapter for ev3dev's sysfs interface."""

    def __init__(self, port, sysfs_root="/sys/class/tacho-motor"):
        self.port = port
        self.path = self._find_motor(sysfs_root, port)

    @staticmethod
    def _find_motor(sysfs_root, port):
        try:
            entries = os.listdir(sysfs_root)
        except OSError as exc:
            raise HardwareError("cannot list motors: {0}".format(exc))

        for entry in entries:
            path = os.path.join(sysfs_root, entry)
            address_path = os.path.join(path, "address")
            try:
                with open(address_path, "r") as address_file:
                    address = address_file.read().strip()
            except (IOError, OSError):
                continue
            if address.endswith(":" + port) or address == port:
                return path

        raise HardwareError("required motor not found on {0}".format(port))

    def _write(self, attribute, value):
        path = os.path.join(self.path, attribute)
        try:
            with open(path, "w") as attribute_file:
                attribute_file.write(str(value))
        except (IOError, OSError) as exc:
            raise HardwareError(
                "failed to write {0} for {1}: {2}".format(attribute, self.port, exc)
            )

    def _read(self, attribute):
        path = os.path.join(self.path, attribute)
        try:
            with open(path, "r") as attribute_file:
                return attribute_file.read().strip()
        except (IOError, OSError) as exc:
            raise HardwareError(
                "failed to read {0} for {1}: {2}".format(attribute, self.port, exc)
            )

    def set_speed(self, speed):
        if speed == 0:
            self.stop()
            return
        self._write("speed_sp", speed)
        self._write("command", "run-forever")

    def stop(self):
        self._write("stop_action", "brake")
        self._write("command", "stop")

    def snapshot(self):
        state = self._read("state")
        return {
            "port": self.port,
            "position": int(self._read("position")),
            "speed": int(self._read("speed")),
            "state": state.split() if state else [],
        }


class MotorController(object):
    """Owns motor state, speed limits, and the stale-command watchdog."""

    def __init__(
        self,
        motors,
        watchdog_seconds=DEFAULT_WATCHDOG_SECONDS,
        drive_speed_limit=DEFAULT_DRIVE_SPEED_LIMIT,
        tool_speed_limit=DEFAULT_TOOL_SPEED_LIMIT,
        clock=None,
    ):
        missing = sorted(set(MOTOR_PORTS) - set(motors))
        if missing:
            raise HardwareError("missing motor roles: {0}".format(", ".join(missing)))
        if watchdog_seconds <= 0:
            raise ValueError("watchdog_seconds must be positive")

        self.motors = motors
        self.watchdog_seconds = float(watchdog_seconds)
        self.drive_speed_limit = int(drive_speed_limit)
        self.tool_speed_limit = int(tool_speed_limit)
        self.clock = clock or time.monotonic
        self.commanded = {"left": 0, "right": 0, "tool": 0}
        self.last_motion_at = None
        self.motion_active = False
        self.last_stop_reason = None
        self.stop_all("startup")

    @staticmethod
    def _normalize_speed(name, value, limit):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError("{0} speed must be a number".format(name))
        if not math.isfinite(value):
            raise ProtocolError("{0} speed must be finite".format(name))
        integer_value = int(round(value))
        return max(-limit, min(limit, integer_value))

    def drive(self, left, right, tool=0):
        applied = {
            "left": self._normalize_speed("left", left, self.drive_speed_limit),
            "right": self._normalize_speed("right", right, self.drive_speed_limit),
            "tool": self._normalize_speed("tool", tool, self.tool_speed_limit),
        }

        if not any(applied.values()):
            self.stop_all("zero-command")
            return applied

        try:
            for role in ("left", "right", "tool"):
                self.motors[role].set_speed(applied[role])
        except Exception:
            self.stop_all("motor-write-failure", suppress_errors=True)
            raise

        self.commanded = applied
        self.last_motion_at = self.clock()
        self.motion_active = True
        self.last_stop_reason = None
        return dict(applied)

    def stop_all(self, reason, suppress_errors=False):
        errors = []
        for role in ("left", "right", "tool"):
            try:
                self.motors[role].stop()
            except Exception as exc:
                errors.append("{0}: {1}".format(role, exc))

        self.commanded = {"left": 0, "right": 0, "tool": 0}
        self.motion_active = False
        self.last_motion_at = None
        self.last_stop_reason = reason

        if errors and not suppress_errors:
            raise HardwareError("failed to stop motors: {0}".format("; ".join(errors)))

    def enforce_watchdog(self):
        if not self.motion_active or self.last_motion_at is None:
            return False
        if self.clock() - self.last_motion_at < self.watchdog_seconds:
            return False
        self.stop_all("watchdog")
        return True

    def status(self):
        snapshots = {}
        for role in ("left", "right", "tool"):
            snapshots[role] = self.motors[role].snapshot()
            snapshots[role]["commanded_speed"] = self.commanded[role]
        return {
            "motors": snapshots,
            "motion_active": self.motion_active,
            "last_stop_reason": self.last_stop_reason,
            "watchdog_timeout_ms": int(self.watchdog_seconds * 1000),
        }


def handle_request(controller, request):
    """Handle one decoded request and return a JSON-serializable response."""

    if not isinstance(request, dict):
        raise ProtocolError("request must be a JSON object")

    command = request.get("command")
    if command == "ping":
        return {
            "status": "ok",
            "message": "pong",
            "protocol_version": PROTOCOL_VERSION,
        }
    if command == "stop":
        controller.stop_all("remote-stop")
        return {"status": "ok", "stopped": True}
    if command == "drive":
        if "left" not in request or "right" not in request:
            raise ProtocolError("drive requires left and right speeds")
        applied = controller.drive(
            request["left"], request["right"], request.get("tool", 0)
        )
        return {"status": "ok", "applied": applied}
    if command == "status":
        response = controller.status()
        response["status"] = "ok"
        return response

    raise ProtocolError("unknown command")


def encode_response(response):
    return (json.dumps(response, separators=(",", ":"), sort_keys=True) + "\n").encode(
        "utf-8"
    )


def decode_request(line):
    try:
        text = line.decode("utf-8")
    except UnicodeDecodeError:
        raise ProtocolError("request must be UTF-8")
    try:
        return json.loads(text)
    except ValueError:
        raise ProtocolError("request must contain valid JSON")


def process_request_line(controller, line):
    """Decode and handle one line, stopping immediately on every error."""

    try:
        request = decode_request(line)
        return handle_request(controller, request)
    except ProtocolError as exc:
        controller.stop_all("protocol-error", suppress_errors=True)
        return {"status": "error", "error": str(exc)}
    except Exception as exc:
        controller.stop_all("request-failure", suppress_errors=True)
        return {"status": "error", "error": str(exc)}


class Ev3JsonServer(object):
    """Single-client TCP server with a watchdog-aware receive loop."""

    def __init__(
        self,
        controller,
        host=DEFAULT_HOST,
        port=DEFAULT_PORT,
        allowed_client=DEFAULT_ALLOWED_CLIENT,
        poll_seconds=DEFAULT_POLL_SECONDS,
        max_line_bytes=DEFAULT_MAX_LINE_BYTES,
    ):
        self.controller = controller
        self.host = host
        self.port = int(port)
        self.allowed_client = allowed_client
        self.poll_seconds = float(poll_seconds)
        self.max_line_bytes = int(max_line_bytes)
        self._stopping = False
        self._listener = None

    def stop(self):
        self._stopping = True
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass

    def serve_forever(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener = listener
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(1)
        listener.settimeout(0.25)
        print("EV3 JSON server listening on {0}:{1}".format(self.host, self.port))

        try:
            while not self._stopping:
                try:
                    connection, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stopping:
                        break
                    raise
                if self.allowed_client and address[0] != self.allowed_client:
                    print("Rejected client from {0}".format(address[0]))
                    connection.close()
                    self.controller.stop_all(
                        "rejected-client", suppress_errors=True
                    )
                    continue
                print("Client connected from {0}:{1}".format(address[0], address[1]))
                self._handle_client(connection)
        finally:
            self.controller.stop_all("server-shutdown", suppress_errors=True)
            try:
                listener.close()
            except OSError:
                pass
            self._listener = None

    def _send_error(self, connection, message):
        connection.sendall(encode_response({"status": "error", "error": message}))

    def _handle_client(self, connection):
        connection.settimeout(self.poll_seconds)
        buffer = b""

        try:
            while not self._stopping:
                self.controller.enforce_watchdog()
                try:
                    data = connection.recv(1024)
                except socket.timeout:
                    continue
                if not data:
                    break
                buffer += data

                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if len(line) > self.max_line_bytes:
                        self._send_error(connection, "request line is too long")
                        return
                    if not line.strip():
                        continue
                    response = process_request_line(self.controller, line)
                    connection.sendall(encode_response(response))

                if len(buffer) > self.max_line_bytes:
                    self._send_error(connection, "request line is too long")
                    return
        finally:
            try:
                connection.close()
            finally:
                self.controller.stop_all("client-disconnect", suppress_errors=True)
                print("Client disconnected; motors stopped")


def build_controller(args):
    motors = {}
    for role, port in MOTOR_PORTS.items():
        motors[role] = SysfsMotor(port, sysfs_root=args.sysfs_root)
    return MotorController(
        motors,
        watchdog_seconds=args.watchdog_ms / 1000.0,
        drive_speed_limit=args.drive_speed_limit,
        tool_speed_limit=args.tool_speed_limit,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--allowed-client", default=DEFAULT_ALLOWED_CLIENT)
    parser.add_argument("--watchdog-ms", type=int, default=500)
    parser.add_argument("--drive-speed-limit", type=int, default=250)
    parser.add_argument("--tool-speed-limit", type=int, default=150)
    parser.add_argument(
        "--sysfs-root", default="/sys/class/tacho-motor", help=argparse.SUPPRESS
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    controller = build_controller(args)
    server = Ev3JsonServer(
        controller,
        host=args.host,
        port=args.port,
        allowed_client=args.allowed_client,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping EV3 server")
    finally:
        server.stop()
        controller.stop_all("process-exit", suppress_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
