#!/usr/bin/env python3
"""Exercise image subscription recovery in isolated localhost ROS domain 188."""
import json
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from image_subscription import ReconnectingImageSubscription

assert os.environ.get('ROS_DOMAIN_ID') == '188'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
node = Node('image_reconnection_test')
seen = []
pub = node.create_publisher(Image, '/test_image_reconnection', 1)
stream = ReconnectingImageSubscription(node, Image, '/test_image_reconnection', lambda m: seen.append(time.monotonic()), 1)
def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(Image())
        rclpy.spin_once(node, timeout_sec=.05)
try:
    pump(2)
    before = len(seen)
    assert before > 0, 'Initial image connection failed'
    node.destroy_subscription(stream.subscription)
    pump(.3)
    broken = len(seen)
    pump(.5)
    assert len(seen) == broken, 'Broken subscription unexpectedly received frames'
    # Advance only the recovery decision's clock to avoid a ten-second idle test.
    assert stream.refresh_if_stale(stream.last_attempt + 11, 11)
    pump(2)
    assert len(seen) > broken, 'No frames received after reconnection'
    print(json.dumps(dict(initial_messages=before, messages_after_reconnect=len(seen)-broken,
                          reconnects=stream.reconnects, passed=True)))
finally:
    node.destroy_node()
    rclpy.shutdown()
