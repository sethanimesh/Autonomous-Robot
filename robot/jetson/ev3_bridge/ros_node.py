#!/usr/bin/env python3
"""ROS 2 /cmd_vel bridge for the Echora EV3 service."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from camera_head import CameraHeadController
from camera_head import CameraHeadError
from ev3_client import Ev3Client
from ev3_client import Ev3ClientError
from kinematics import twist_to_motor_speeds
from odometry import DifferentialOdometry


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
        self.declare_parameter("encoder_counts_per_rev", 360)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("camera_head_calibrated", False)
        self.declare_parameter("camera_head_forward_position", 0)
        self.declare_parameter("camera_head_down_position", 0)
        self.declare_parameter("camera_head_minimum_position", -180)
        self.declare_parameter("camera_head_maximum_position", 180)
        self.declare_parameter("camera_head_speed", 40)
        self.declare_parameter("camera_head_home_speed", 60)
        self.declare_parameter("camera_head_max_jog_degrees", 15)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.left_sign = int(self.get_parameter("left_sign").value)
        self.right_sign = int(self.get_parameter("right_sign").value)
        self.max_motor_speed = int(self.get_parameter("max_motor_speed").value)
        self.encoder_counts_per_rev = int(
            self.get_parameter("encoder_counts_per_rev").value
        )
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
        self.camera_head = CameraHeadController(
            self.client,
            calibrated=bool(self.get_parameter("camera_head_calibrated").value),
            forward_position=int(
                self.get_parameter("camera_head_forward_position").value
            ),
            down_position=int(self.get_parameter("camera_head_down_position").value),
            minimum_position=int(
                self.get_parameter("camera_head_minimum_position").value
            ),
            maximum_position=int(
                self.get_parameter("camera_head_maximum_position").value
            ),
            speed=int(self.get_parameter("camera_head_speed").value),
            home_speed=int(self.get_parameter("camera_head_home_speed").value),
            max_jog_degrees=int(
                self.get_parameter("camera_head_max_jog_degrees").value
            ),
        )
        self.last_command = None
        self.last_command_time = None
        self.motion_active = False
        self.tick_count = 0
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.odometry = DifferentialOdometry(
            self.wheel_radius_m,
            self.track_width_m,
            self.encoder_counts_per_rev,
            self.left_sign,
            self.right_sign,
        )

        self.status_publisher = self.create_publisher(String, "/robot_status", 10)
        self.camera_head_status_publisher = self.create_publisher(
            String, "/camera_head/status", 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            String, "/camera_head/command", self.on_camera_head_command, 10
        )
        self.create_timer(1.0 / update_rate_hz, self.on_timer)
        self.get_logger().info("EV3 bridge ready; waiting for /cmd_vel")

    def on_cmd_vel(self, message):
        self.last_command = (float(message.linear.x), float(message.angular.z))
        self.last_command_time = time.monotonic()

    def on_camera_head_command(self, message):
        # Never let a recently received chassis command resume after a head move.
        self.last_command = None
        self.last_command_time = None
        self.motion_active = False
        try:
            self.camera_head.execute(message.data)
            self.publish_status(self.client.status())
        except (CameraHeadError, Ev3ClientError, ValueError) as exc:
            self.get_logger().error("camera-head command rejected: {0}".format(exc))
            try:
                self.client.stop()
            except Ev3ClientError:
                self.client.close()

    def publish_status(self, status):
        if not rclpy.ok():
            return
        self.publish_odometry(status)
        message = String()
        message.data = json.dumps(status, separators=(",", ":"), sort_keys=True)
        self.status_publisher.publish(message)
        head_message = String()
        head_message.data = json.dumps(
            self.camera_head.describe(status), separators=(",", ":"), sort_keys=True
        )
        self.camera_head_status_publisher.publish(head_message)

    def publish_odometry(self, status):
        motors = status.get("motors")
        if not motors or "left" not in motors or "right" not in motors:
            return

        now = self.get_clock().now()
        stamp = now.to_msg()
        state = self.odometry.update(
            motors["left"]["position"],
            motors["right"]["position"],
            now.nanoseconds / 1000000000.0,
        )
        half_heading = state["heading"] / 2.0
        orientation_z = math.sin(half_heading)
        orientation_w = math.cos(half_heading)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = state["x"]
        odom.pose.pose.position.y = state["y"]
        odom.pose.pose.orientation.z = orientation_z
        odom.pose.pose.orientation.w = orientation_w
        odom.twist.twist.linear.x = state["linear_velocity"]
        odom.twist.twist.angular.z = state["angular_velocity"]
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[14] = 1000000.0
        odom.twist.covariance[21] = 1000000.0
        odom.twist.covariance[28] = 1000000.0
        odom.twist.covariance[35] = 0.2
        self.odom_publisher.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_track_joint", "right_track_joint"]
        joints.position = [
            state["left_joint_position"],
            state["right_joint_position"],
        ]
        joints.velocity = [
            math.radians(motors["left"]["speed"]) * self.left_sign,
            math.radians(motors["right"]["speed"]) * self.right_sign,
        ]
        self.joint_state_publisher.publish(joints)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = state["x"]
            transform.transform.translation.y = state["y"]
            transform.transform.rotation.z = orientation_z
            transform.transform.rotation.w = orientation_w
            self.tf_broadcaster.sendTransform(transform)

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
            elif self.tick_count % 5 == 0:
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
            response = self.client.drive(left_speed, right_speed)
            self.motion_active = bool(left_speed or right_speed)
            if self.tick_count % 2 == 0:
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
