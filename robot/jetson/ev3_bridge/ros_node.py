#!/usr/bin/env python3
"""ROS 2 /cmd_vel bridge for the Echora EV3 service."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ev3_client import Ev3Client
from ev3_client import Ev3ClientError
from kinematics import twist_to_motor_speeds


class Ev3BridgeNode(Node):
    def __init__(self):
        super().__init__("ev3_bridge")
        self.declare_parameter("ev3_host", "192.168.1.25")
        self.declare_parameter("ev3_port", 9999)
        self.declare_parameter("wheel_radius_m", 0.03)
        self.declare_parameter("track_width_m", 0.12)
        self.declare_parameter("left_sign", 1)
        self.declare_parameter("right_sign", 1)
        self.declare_parameter("max_motor_speed", 120)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("update_rate_hz", 10.0)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.left_sign = int(self.get_parameter("left_sign").value)
        self.right_sign = int(self.get_parameter("right_sign").value)
        self.max_motor_speed = int(self.get_parameter("max_motor_speed").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        if self.command_timeout_sec <= 0 or self.command_timeout_sec >= 0.5:
            raise ValueError("command_timeout_sec must be greater than 0 and below 0.5")
        if update_rate_hz <= 0:
            raise ValueError("update_rate_hz must be positive")

        self.client = Ev3Client(
            host=str(self.get_parameter("ev3_host").value),
            port=int(self.get_parameter("ev3_port").value),
            timeout_seconds=1.0,
        )
        self.last_command = None
        self.last_command_time = None
        self.motion_active = False
        self.tick_count = 0

        self.status_publisher = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(1.0 / update_rate_hz, self.on_timer)
        self.get_logger().info("EV3 bridge ready; waiting for /cmd_vel")

    def on_cmd_vel(self, message):
        self.last_command = (float(message.linear.x), float(message.angular.z))
        self.last_command_time = time.monotonic()

    def publish_status(self, status):
        if not rclpy.ok():
            return
        message = String()
        message.data = json.dumps(status, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)

    def stop_for_reason(self, reason):
        try:
            response = self.client.stop()
            response["bridge_stop_reason"] = reason
            self.publish_status(response)
        except Ev3ClientError as exc:
            self.get_logger().error("EV3 stop failed: {0}".format(exc))
            self.client.close()
        self.motion_active = False

    def on_timer(self):
        self.tick_count += 1
        now = time.monotonic()
        command_fresh = (
            self.last_command is not None
            and self.last_command_time is not None
            and now - self.last_command_time <= self.command_timeout_sec
        )

        if not command_fresh:
            if self.motion_active:
                self.stop_for_reason("cmd_vel_timeout")
            elif self.tick_count % 10 == 0:
                try:
                    self.publish_status(self.client.status())
                except Ev3ClientError as exc:
                    self.get_logger().error("EV3 status failed: {0}".format(exc))
                    self.client.close()
            return

        linear_mps, angular_rps = self.last_command
        left_speed, right_speed = twist_to_motor_speeds(
            linear_mps,
            angular_rps,
            self.wheel_radius_m,
            self.track_width_m,
            self.left_sign,
            self.right_sign,
            self.max_motor_speed,
        )

        try:
            response = self.client.drive(left_speed, right_speed, 0)
            self.motion_active = bool(left_speed or right_speed)
            if self.tick_count % 5 == 0:
                status = self.client.status()
                status["bridge_applied"] = response.get("applied", {})
                self.publish_status(status)
        except Ev3ClientError as exc:
            self.get_logger().error("EV3 command failed: {0}".format(exc))
            self.client.close()
            self.motion_active = False

    def shutdown(self):
        self.stop_for_reason("node_shutdown")
        self.client.close()


def main(args=None):
    rclpy.init(args=args)
    node = Ev3BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
