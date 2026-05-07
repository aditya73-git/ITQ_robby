from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_share = Path(get_package_share_directory("robby_control"))
    description_share = Path(get_package_share_directory("robby_description"))

    ackermann_config = control_share / "config" / "ackermann_cmd.yaml"
    bridge_config = control_share / "config" / "ackermann_joint_state_bridge.yaml"
    odometry_config = control_share / "config" / "swerve_odometry.yaml"
    display_launch = description_share / "launch" / "display.launch.py"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(display_launch)),
                launch_arguments={"use_joint_state_publisher_gui": "false"}.items(),
            ),
            Node(
                package="robby_control",
                executable="ackermann_cmd_node",
                name="ackermann_cmd_node",
                parameters=[str(ackermann_config)],
            ),
            Node(
                package="robby_control",
                executable="ackermann_joint_state_bridge",
                name="ackermann_joint_state_bridge",
                parameters=[str(bridge_config)],
            ),
            Node(
                package="robby_control",
                executable="swerve_odometry_node",
                name="swerve_odometry_node",
                parameters=[str(odometry_config)],
            ),
        ]
    )
