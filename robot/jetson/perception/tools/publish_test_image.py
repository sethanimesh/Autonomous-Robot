#!/usr/bin/env python3
"""Temporarily publish a test photograph as ROS bgr8 frames; saves nothing."""

import argparse
import array
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args(argv)
    if not 0.0 < args.rate <= 120.0:
        raise SystemExit("--rate must be greater than 0 and no more than 120")
    if not 0.0 < args.seconds <= 60.0:
        raise SystemExit("--seconds must be greater than 0 and no more than 60")
    source = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if source is None:
        raise SystemExit("cannot read {0}".format(args.image))
    frame = cv2.resize(source, (640, 480), interpolation=cv2.INTER_AREA)
    rclpy.init()
    node = Node("echora_test_image_publisher")
    publisher = node.create_publisher(Image, args.topic, 10)
    started = time.monotonic()
    period = 1.0 / args.rate
    try:
        while time.monotonic() - started < args.seconds:
            message = Image()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "temporary_test_image"
            message.height = 480
            message.width = 640
            message.encoding = "bgr8"
            message.is_bigendian = False
            message.step = 640 * 3
            message.data = array.array("B", frame.reshape(-1))
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
