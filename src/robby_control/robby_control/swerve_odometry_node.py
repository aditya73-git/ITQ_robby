import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half_yaw)
    quat.w = math.cos(half_yaw)
    return quat


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def solve_3x3(matrix: List[List[float]], vector: List[float]) -> List[float]:
    augmented = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    size = 3

    for pivot_index in range(size):
        pivot_row = max(range(pivot_index, size), key=lambda row: abs(augmented[row][pivot_index]))
        pivot_value = augmented[pivot_row][pivot_index]
        if abs(pivot_value) < 1e-9:
            raise ValueError("Singular least-squares system")

        if pivot_row != pivot_index:
            augmented[pivot_index], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_index]

        pivot_value = augmented[pivot_index][pivot_index]
        for column in range(pivot_index, size + 1):
            augmented[pivot_index][column] /= pivot_value

        for row in range(size):
            if row == pivot_index:
                continue
            factor = augmented[row][pivot_index]
            for column in range(pivot_index, size + 1):
                augmented[row][column] -= factor * augmented[pivot_index][column]

    return [augmented[index][size] for index in range(size)]


def least_squares_body_delta(rows: Sequence[Tuple[float, float, float]], travel: Sequence[float]) -> Tuple[float, float, float]:
    ata = [[0.0, 0.0, 0.0] for _ in range(3)]
    atb = [0.0, 0.0, 0.0]

    for row, distance in zip(rows, travel):
        for i in range(3):
            atb[i] += row[i] * distance
            for j in range(3):
                ata[i][j] += row[i] * row[j]

    return tuple(solve_3x3(ata, atb))


@dataclass(frozen=True)
class ModuleGeometry:
    x: float
    y: float


class SwerveOdometryNode(Node):
    ODOMETRY_FAILURE_ERROR_THRESHOLD = 50

    def __init__(self) -> None:
        super().__init__("swerve_odometry_node")

        self.declare_parameter(
            "drive_joint_names",
            ["f_leftwheel", "b_leftwheel", "b_rightwheel", "f_rightwheel"],
        )
        self.declare_parameter(
            "steer_joint_names",
            ["f_left_steer", "b_leftsteer", "b_rightsteer", "f_right_steer"],
        )
        self.declare_parameter(
            "wheel_positions",
            [
                0.2014,
                0.16556,
                -0.20473,
                0.16556,
                -0.2046,
                -0.16556,
                0.20127,
                -0.16556,
            ],
        )
        self.declare_parameter("wheel_radius", 0.052)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.drive_joint_names = list(self.get_parameter("drive_joint_names").value)
        self.steer_joint_names = list(self.get_parameter("steer_joint_names").value)
        wheel_positions = list(self.get_parameter("wheel_positions").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self._validate_parameters(wheel_positions)
        self.modules = [
            ModuleGeometry(x=wheel_positions[index], y=wheel_positions[index + 1])
            for index in range(0, len(wheel_positions), 2)
        ]

        self.previous_drive_positions: Dict[str, float] = {}
        self.previous_stamp = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.missing_joint_warning_sent = False
        self.odometry_failure_count = 0
        self.odometry_failure_error_logged = False

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            50,
        )

        self.get_logger().info(
            "Swerve odometry node ready. Waiting for joint states on %s" % self.joint_state_topic
        )

    def _validate_parameters(self, wheel_positions: Sequence[float]) -> None:
        if len(self.drive_joint_names) != 4 or len(self.steer_joint_names) != 4:
            raise ValueError("Expected exactly 4 drive joints and 4 steer joints")
        if len(wheel_positions) != 8:
            raise ValueError("wheel_positions must contain 8 values: x1 y1 x2 y2 x3 y3 x4 y4")

    def joint_state_callback(self, message: JointState) -> None:
        joint_indices = {name: index for index, name in enumerate(message.name)}
        required_joints = self.drive_joint_names + self.steer_joint_names
        missing = [name for name in required_joints if name not in joint_indices]
        if missing:
            if not self.missing_joint_warning_sent:
                self.get_logger().warning(
                    "Waiting for required joints in /joint_states: %s" % ", ".join(missing)
                )
                self.missing_joint_warning_sent = True
            return

        self.missing_joint_warning_sent = False

        drive_positions = {
            name: float(message.position[joint_indices[name]]) for name in self.drive_joint_names
        }
        steer_positions = {
            name: float(message.position[joint_indices[name]]) for name in self.steer_joint_names
        }

        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        if not self.previous_drive_positions:
            self.previous_drive_positions = drive_positions
            self.previous_stamp = stamp
            return

        dt = (
            (stamp.sec - self.previous_stamp.sec)
            + (stamp.nanosec - self.previous_stamp.nanosec) * 1e-9
        )
        if dt <= 0.0:
            return

        rows = []
        wheel_travel = []
        for drive_name, steer_name, module in zip(
            self.drive_joint_names,
            self.steer_joint_names,
            self.modules,
        ):
            delta_rotation = drive_positions[drive_name] - self.previous_drive_positions[drive_name]
            distance = delta_rotation * self.wheel_radius
            steer_angle = steer_positions[steer_name]
            cos_angle = math.cos(steer_angle)
            sin_angle = math.sin(steer_angle)
            rows.append(
                (
                    cos_angle,
                    sin_angle,
                    (-module.y * cos_angle) + (module.x * sin_angle),
                )
            )
            wheel_travel.append(distance)

        try:
            delta_x_body, delta_y_body, delta_yaw = least_squares_body_delta(rows, wheel_travel)
        except ValueError as error:
            self.odometry_failure_count += 1
            if (
                self.odometry_failure_count >= self.ODOMETRY_FAILURE_ERROR_THRESHOLD
                and not self.odometry_failure_error_logged
            ):
                self.get_logger().error(
                    "Odometry solve failed %d consecutive times; latest error: %s"
                    % (self.odometry_failure_count, error)
                )
                self.odometry_failure_error_logged = True
            self.previous_drive_positions = drive_positions
            self.previous_stamp = stamp
            return

        self.odometry_failure_count = 0
        self.odometry_failure_error_logged = False

        heading = self.pose_yaw + 0.5 * delta_yaw
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        self.pose_x += (cos_heading * delta_x_body) - (sin_heading * delta_y_body)
        self.pose_y += (sin_heading * delta_x_body) + (cos_heading * delta_y_body)
        self.pose_yaw = normalize_angle(self.pose_yaw + delta_yaw)

        linear_x = delta_x_body / dt
        linear_y = delta_y_body / dt
        angular_z = delta_yaw / dt

        self.publish_odometry(stamp, linear_x, linear_y, angular_z)

        self.previous_drive_positions = drive_positions
        self.previous_stamp = stamp

    def publish_odometry(self, stamp, linear_x: float, linear_y: float, angular_z: float) -> None:
        quaternion = yaw_to_quaternion(self.pose_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.08

        self.odom_publisher.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwerveOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
