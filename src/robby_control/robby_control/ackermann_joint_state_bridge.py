from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AckermannJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("ackermann_joint_state_bridge")

        self.declare_parameter("command_topic", "/ackermann_cmd_joint_states")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter(
            "joint_names",
            [
                "f_left_steer",
                "f_right_steer",
                "b_leftsteer",
                "b_rightsteer",
                "f_leftwheel",
                "f_rightwheel",
                "b_leftwheel",
                "b_rightwheel",
            ],
        )

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)

        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.velocities: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.last_update_time = self.get_clock().now()

        self.publisher = self.create_publisher(JointState, self.joint_state_topic, 20)
        self.subscription = self.create_subscription(
            JointState,
            self.command_topic,
            self.command_callback,
            20,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_joint_states)

        self.get_logger().info(
            "Ackermann joint-state bridge ready. Commands: %s -> Joint states: %s"
            % (self.command_topic, self.joint_state_topic)
        )

    def command_callback(self, message: JointState) -> None:
        name_to_index = {name: index for index, name in enumerate(message.name)}

        for joint_name in self.joint_names:
            if joint_name not in name_to_index:
                continue

            index = name_to_index[joint_name]
            if index < len(message.position):
                self.positions[joint_name] = float(message.position[index])
            if index < len(message.velocity):
                self.velocities[joint_name] = float(message.velocity[index])

    def publish_joint_states(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt < 0.0:
            dt = 0.0

        for joint_name, velocity in self.velocities.items():
            if "wheel" in joint_name:
                self.positions[joint_name] += velocity * dt

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [self.positions[name] for name in self.joint_names]
        joint_state.velocity = [self.velocities[name] for name in self.joint_names]

        self.publisher.publish(joint_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
