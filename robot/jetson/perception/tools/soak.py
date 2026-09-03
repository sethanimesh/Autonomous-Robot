#!/usr/bin/env python3
"""Watch a running detector for memory growth, staleness, and errors.

    python3 soak.py --minutes 10

Samples the node's resident memory and CPU from /proc alongside the rates and
latency it reports on /perception/status. Three things decide whether a long
run is healthy:

    RSS        must plateau rather than climb continuously
    age_s      must not trend upward, or published frames are going stale
    errs       must stay flat

Input rate is expected to vary: it follows the camera, which slows in dimmer
light. A dip there is not a detector fault, and the detector recovering from
one without error is the point of watching.
"""

import argparse
import json
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String

STATUS_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
CLOCK_TICKS = os.sysconf("SC_CLK_TCK")


def find_detector_pid(pattern="person_detector[.]py"):
    """Return the python process running the detector, not its shell wrapper."""
    found = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    for pid in found.stdout.split():
        try:
            with open("/proc/{0}/cmdline".format(pid), "rb") as handle:
                if handle.read().split(b"\0")[0].endswith(b"python3"):
                    return pid
        except OSError:
            continue
    return None


def resident_kb(pid):
    try:
        with open("/proc/{0}/status".format(pid)) as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def cpu_ticks(pid):
    try:
        with open("/proc/{0}/stat".format(pid)) as handle:
            fields = handle.read().split()
        return int(fields[13]) + int(fields[14])
    except (OSError, IndexError):
        return 0


class StatusReader(Node):
    def __init__(self, topic):
        super().__init__("echora_soak")
        self.payload = {}
        self.create_subscription(String, topic, self.on_status, STATUS_QOS)

    def on_status(self, message):
        try:
            self.payload = json.loads(message.data)
        except ValueError:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--minutes", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=30.0)
    parser.add_argument("--status-topic", default="/perception/status")
    args = parser.parse_args(argv)

    pid = find_detector_pid()
    if pid is None:
        raise SystemExit("no running person_detector.py found")

    rclpy.init()
    reader = StatusReader(args.status_topic)
    duration = args.minutes * 60.0
    print("soaking {0:.0f} min, detector pid={1}".format(args.minutes, pid), flush=True)
    header = "{0:<7} {1:<9} {2:<7} {3:<8} {4:<8} {5:<9} {6:<9} {7:<8} {8:<5}".format(
        "t(s)", "RSS(kB)", "CPU%", "in_Hz", "inf_Hz", "mean_ms", "p95_ms", "age_s", "errs"
    )
    print(header, flush=True)

    started = time.time()
    last_sample = started
    last_ticks = cpu_ticks(pid)
    rows = []
    try:
        while time.time() - started < duration:
            rclpy.spin_once(reader, timeout_sec=0.2)
            if time.time() - last_sample < args.interval:
                continue
            now = time.time()
            ticks = cpu_ticks(pid)
            cpu = 100.0 * (ticks - last_ticks) / CLOCK_TICKS / (now - last_sample)
            last_ticks, last_sample = ticks, now

            status = reader.payload
            rss = resident_kb(pid)
            if rss == 0:
                print("detector process is gone", flush=True)
                break
            age = status.get("last_frame_age_sec")
            errors = status.get("inference_errors")
            rows.append((now - started, rss, age, errors))
            print(
                "{0:<7.0f} {1:<9d} {2:<7.1f} {3:<8.2f} {4:<8.2f} {5:<9.2f} {6:<9.2f} "
                "{7:<8} {8:<5}".format(
                    now - started,
                    rss,
                    cpu,
                    status.get("input_rate_hz", 0.0),
                    status.get("inference_rate_hz", 0.0),
                    status.get("mean_latency_ms", 0.0),
                    status.get("p95_latency_ms", 0.0),
                    age,
                    errors,
                ),
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        print("\n=== soak summary ===")
        if rows:
            first, last = rows[0][1], rows[-1][1]
            print(
                "RSS {0} kB -> {1} kB (delta {2:+d} kB, {3:+.2f}%); min {4} max {5}".format(
                    first,
                    last,
                    last - first,
                    100.0 * (last - first) / max(1, first),
                    min(r[1] for r in rows),
                    max(r[1] for r in rows),
                )
            )
            ages = [r[2] for r in rows if r[2] is not None]
            if ages:
                print(
                    "frame age min {0:.3f} max {1:.3f} last {2:.3f} "
                    "(must not trend upward)".format(min(ages), max(ages), ages[-1])
                )
            print("inference errors {0} -> {1}".format(rows[0][3], rows[-1][3]))
        status = reader.payload
        print(
            "final: state={0} provider={1} inferences={2} dropped={3} rejected={4}".format(
                status.get("state"),
                status.get("provider"),
                status.get("inferences"),
                status.get("frames_dropped"),
                status.get("frames_rejected"),
            )
        )
        reader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
