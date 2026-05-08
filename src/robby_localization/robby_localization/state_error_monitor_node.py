import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(message) -> float:
    siny_cosp = 2.0 * ((message.w * message.z) + (message.x * message.y))
    cosy_cosp = 1.0 - (2.0 * ((message.y * message.y) + (message.z * message.z)))
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StateErrorMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_error_monitor_node")

        self.declare_parameter("reference_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/odometry/filtered")
        self.declare_parameter("error_topic", "/state_estimation/error")
        self.declare_parameter("report_period", 2.0)

        self.reference_topic = str(self.get_parameter("reference_topic").value)
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.report_period = float(self.get_parameter("report_period").value)

        self.latest_reference: Optional[Odometry] = None
        self.latest_estimate: Optional[Odometry] = None
        self.error_publisher = self.create_publisher(Vector3Stamped, self.error_topic, 20)
        self.reference_subscriber = self.create_subscription(
            Odometry, self.reference_topic, self.reference_callback, 20
        )
        self.estimate_subscriber = self.create_subscription(
            Odometry, self.estimate_topic, self.estimate_callback, 20
        )
        self.report_timer = self.create_timer(self.report_period, self.report_error)

        self.get_logger().info(
            "State error monitor ready. reference=%s estimate=%s error=%s"
            % (self.reference_topic, self.estimate_topic, self.error_topic)
        )

    def reference_callback(self, message: Odometry) -> None:
        self.latest_reference = message
        self.publish_error_if_ready(message.header.stamp)

    def estimate_callback(self, message: Odometry) -> None:
        self.latest_estimate = message
        self.publish_error_if_ready(message.header.stamp)

    def publish_error_if_ready(self, stamp) -> None:
        if self.latest_reference is None or self.latest_estimate is None:
            return

        reference_pose = self.latest_reference.pose.pose
        estimate_pose = self.latest_estimate.pose.pose
        error_message = Vector3Stamped()
        error_message.header.stamp = stamp
        error_message.header.frame_id = "ground_truth"
        error_message.vector.x = estimate_pose.position.x - reference_pose.position.x
        error_message.vector.y = estimate_pose.position.y - reference_pose.position.y
        error_message.vector.z = normalize_angle(
            yaw_from_quaternion(estimate_pose.orientation)
            - yaw_from_quaternion(reference_pose.orientation)
        )
        self.error_publisher.publish(error_message)

    def report_error(self) -> None:
        if self.latest_reference is None or self.latest_estimate is None:
            return

        dx = self.latest_estimate.pose.pose.position.x - self.latest_reference.pose.pose.position.x
        dy = self.latest_estimate.pose.pose.position.y - self.latest_reference.pose.pose.position.y
        yaw_error = normalize_angle(
            yaw_from_quaternion(self.latest_estimate.pose.pose.orientation)
            - yaw_from_quaternion(self.latest_reference.pose.pose.orientation)
        )
        position_error = math.hypot(dx, dy)
        self.get_logger().info(
            "Estimation error: position=%.3f m, dx=%.3f m, dy=%.3f m, yaw=%.3f rad"
            % (position_error, dx, dy, yaw_error)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateErrorMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
