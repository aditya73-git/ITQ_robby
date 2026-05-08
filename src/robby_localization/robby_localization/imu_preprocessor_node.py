from copy import deepcopy
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def covariance_is_unset(values: Iterable[float]) -> bool:
    return all(abs(value) < 1e-12 for value in values)


class ImuPreprocessorNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_preprocessor_node")

        self.declare_parameter("input_topic", "/imu/data")
        self.declare_parameter("output_topic", "/imu/data/filtered")
        self.declare_parameter("frame_id", "imu_link")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.publisher = self.create_publisher(Imu, self.output_topic, 20)
        self.subscription = self.create_subscription(Imu, self.input_topic, self.imu_callback, 50)

        self.get_logger().info(
            "IMU preprocessor ready. %s -> %s with frame_id=%s"
            % (self.input_topic, self.output_topic, self.frame_id)
        )

    def imu_callback(self, message: Imu) -> None:
        processed = deepcopy(message)
        processed.header.frame_id = self.frame_id

        if covariance_is_unset(processed.orientation_covariance):
            processed.orientation_covariance = [
                0.02,
                0.0,
                0.0,
                0.0,
                0.02,
                0.0,
                0.0,
                0.0,
                0.04,
            ]

        if covariance_is_unset(processed.angular_velocity_covariance):
            processed.angular_velocity_covariance = [
                0.001,
                0.0,
                0.0,
                0.0,
                0.001,
                0.0,
                0.0,
                0.0,
                0.002,
            ]

        if covariance_is_unset(processed.linear_acceleration_covariance):
            processed.linear_acceleration_covariance = [
                0.04,
                0.0,
                0.0,
                0.0,
                0.04,
                0.0,
                0.0,
                0.0,
                0.06,
            ]

        self.publisher.publish(processed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuPreprocessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
