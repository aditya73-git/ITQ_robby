import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AckermannCmdNode(Node):
    def __init__(self) -> None:
        super().__init__("ackermann_cmd_node")

        self.declare_parameter("wheelbase", 0.406)
        self.declare_parameter("track_width", 0.33112)
        self.declare_parameter("wheel_radius", 0.052)
        self.declare_parameter("max_steer_angle", 0.8)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/ackermann_cmd_joint_states")
        self.declare_parameter(
            "steering_joint_names",
            ["f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"],
        )
        self.declare_parameter(
            "drive_joint_names",
            ["f_leftwheel", "f_rightwheel", "b_leftwheel", "b_rightwheel"],
        )

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_steer_angle = float(self.get_parameter("max_steer_angle").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.steering_joint_names: List[str] = list(self.get_parameter("steering_joint_names").value)
        self.drive_joint_names: List[str] = list(self.get_parameter("drive_joint_names").value)

        if len(self.steering_joint_names) != 4 or len(self.drive_joint_names) != 4:
            raise ValueError("Ackermann command node expects 4 steering joints and 4 drive joints")

        self.last_lateral_warning_time = None

        self.command_publisher = self.create_publisher(JointState, self.command_topic, 10)
        self.cmd_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20,
        )

        self.get_logger().info(
            "Ackermann command node ready. Listening on %s and publishing %s"
            % (self.cmd_vel_topic, self.command_topic)
        )

    def cmd_callback(self, message: Twist) -> None:
        vx = float(message.linear.x)
        vy = float(message.linear.y)
        wz = float(message.angular.z)

        if abs(vy) > 1e-4:
            now = self.get_clock().now().nanoseconds
            if self.last_lateral_warning_time is None or (now - self.last_lateral_warning_time) > 2e9:
                self.get_logger().warning(
                    "Ackermann mode ignores lateral cmd_vel.y. Received %.4f and using 0.0 instead." % vy
                )
                self.last_lateral_warning_time = now

        left_steer = 0.0
        right_steer = 0.0
        front_left_speed = 0.0
        front_right_speed = 0.0
        rear_left_speed = 0.0
        rear_right_speed = 0.0

        if abs(vx) < 1e-6 and abs(wz) < 1e-6:
            pass
        elif abs(vx) < 1e-6:
            self.get_logger().warning(
                "Pure rotation is not supported in this simple Ackermann mode. Publishing zero command."
            )
        elif abs(wz) < 1e-6:
            wheel_angular_velocity = vx / self.wheel_radius
            front_left_speed = wheel_angular_velocity
            front_right_speed = wheel_angular_velocity
            rear_left_speed = wheel_angular_velocity
            rear_right_speed = wheel_angular_velocity
        else:
            travel_sign = 1.0 if vx >= 0.0 else -1.0
            turn_left = wz > 0.0
            turn_radius = abs(vx / wz)
            half_track = self.track_width * 0.5

            inner_radius = max(turn_radius - half_track, 1e-6)
            outer_radius = turn_radius + half_track
            inner_steer = math.atan2(self.wheelbase, inner_radius)
            outer_steer = math.atan2(self.wheelbase, outer_radius)

            if turn_left:
                left_steer = inner_steer
                right_steer = outer_steer
                left_radius = inner_radius
                right_radius = outer_radius
            else:
                left_steer = -outer_steer
                right_steer = -inner_steer
                left_radius = outer_radius
                right_radius = inner_radius

            left_steer = max(-self.max_steer_angle, min(self.max_steer_angle, left_steer))
            right_steer = max(-self.max_steer_angle, min(self.max_steer_angle, right_steer))

            turn_rate = abs(wz)
            rear_left_linear = travel_sign * turn_rate * left_radius
            rear_right_linear = travel_sign * turn_rate * right_radius
            front_left_linear = travel_sign * turn_rate * math.sqrt(
                (left_radius * left_radius) + (self.wheelbase * self.wheelbase)
            )
            front_right_linear = travel_sign * turn_rate * math.sqrt(
                (right_radius * right_radius) + (self.wheelbase * self.wheelbase)
            )

            front_left_speed = front_left_linear / self.wheel_radius
            front_right_speed = front_right_linear / self.wheel_radius
            rear_left_speed = rear_left_linear / self.wheel_radius
            rear_right_speed = rear_right_linear / self.wheel_radius

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = self.steering_joint_names + self.drive_joint_names
        command.position = [left_steer, right_steer, 0.0, 0.0] + [0.0] * len(self.drive_joint_names)
        command.velocity = [0.0] * len(self.steering_joint_names) + [
            front_left_speed,
            front_right_speed,
            rear_left_speed,
            rear_right_speed,
        ]

        self.command_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannCmdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
