from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robby_control"))
    config_path = package_share / "config" / "swerve_odometry.yaml"

    return LaunchDescription(
        [
            Node(
                package="robby_control",
                executable="swerve_odometry_node",
                name="swerve_odometry_node",
                parameters=[str(config_path)],
            ),
        ]
    )
