from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robby_control"))
    config_path = package_share / "config" / "ackermann_cmd.yaml"

    return LaunchDescription(
        [
            Node(
                package="robby_control",
                executable="ackermann_cmd_node",
                name="ackermann_cmd_node",
                parameters=[str(config_path)],
            ),
        ]
    )
