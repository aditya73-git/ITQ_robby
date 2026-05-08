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


def relative_pose_2d(
    x: float,
    y: float,
    yaw: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> tuple[float, float, float]:
    dx = x - origin_x
    dy = y - origin_y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    rel_x = (cos_yaw * dx) + (sin_yaw * dy)
    rel_y = (-sin_yaw * dx) + (cos_yaw * dy)
    rel_yaw = normalize_angle(yaw - origin_yaw)
    return rel_x, rel_y, rel_yaw


class StateErrorMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_error_monitor_node")

        self.declare_parameter("reference_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/odometry/filtered")
        self.declare_parameter("error_topic", "/state_estimation/error")
        self.declare_parameter("report_period", 2.0)
        self.declare_parameter("align_initial_pose", True)

        self.reference_topic = str(self.get_parameter("reference_topic").value)
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.error_topic = str(self.get_parameter("error_topic").value)
        self.report_period = float(self.get_parameter("report_period").value)
        self.align_initial_pose = bool(self.get_parameter("align_initial_pose").value)

        self.latest_reference: Optional[Odometry] = None
        self.latest_estimate: Optional[Odometry] = None
        self.reference_origin: Optional[tuple[float, float, float]] = None
        self.estimate_origin: Optional[tuple[float, float, float]] = None
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
        if self.align_initial_pose and self.reference_origin is None:
            self.reference_origin = self.pose_from_odometry(message)
        self.publish_error_if_ready(message.header.stamp)

    def estimate_callback(self, message: Odometry) -> None:
        self.latest_estimate = message
        if self.align_initial_pose and self.estimate_origin is None:
            self.estimate_origin = self.pose_from_odometry(message)
        self.publish_error_if_ready(message.header.stamp)

    def pose_from_odometry(self, message: Odometry) -> tuple[float, float, float]:
        pose = message.pose.pose
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def publish_error_if_ready(self, stamp) -> None:
        if self.latest_reference is None or self.latest_estimate is None:
            return

        reference_x, reference_y, reference_yaw = self.pose_from_odometry(self.latest_reference)
        estimate_x, estimate_y, estimate_yaw = self.pose_from_odometry(self.latest_estimate)

        if self.align_initial_pose:
            if self.reference_origin is None or self.estimate_origin is None:
                return
            reference_x, reference_y, reference_yaw = relative_pose_2d(
                reference_x,
                reference_y,
                reference_yaw,
                *self.reference_origin,
            )
            estimate_x, estimate_y, estimate_yaw = relative_pose_2d(
                estimate_x,
                estimate_y,
                estimate_yaw,
                *self.estimate_origin,
            )

        error_message = Vector3Stamped()
        error_message.header.stamp = stamp
        error_message.header.frame_id = "ground_truth_aligned" if self.align_initial_pose else "ground_truth"
        error_message.vector.x = estimate_x - reference_x
        error_message.vector.y = estimate_y - reference_y
        error_message.vector.z = normalize_angle(estimate_yaw - reference_yaw)
        self.error_publisher.publish(error_message)

    def report_error(self) -> None:
        if self.latest_reference is None or self.latest_estimate is None:
            return

        reference_x, reference_y, reference_yaw = self.pose_from_odometry(self.latest_reference)
        estimate_x, estimate_y, estimate_yaw = self.pose_from_odometry(self.latest_estimate)

        if self.align_initial_pose:
            if self.reference_origin is None or self.estimate_origin is None:
                return
            reference_x, reference_y, reference_yaw = relative_pose_2d(
                reference_x,
                reference_y,
                reference_yaw,
                *self.reference_origin,
            )
            estimate_x, estimate_y, estimate_yaw = relative_pose_2d(
                estimate_x,
                estimate_y,
                estimate_yaw,
                *self.estimate_origin,
            )

        dx = estimate_x - reference_x
        dy = estimate_y - reference_y
        yaw_error = normalize_angle(estimate_yaw - reference_yaw)
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
