from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = Path(get_package_share_directory("robby_localization"))

    localization_input = Node(
        package="robby_localization",
        executable="localization_input_node",
        name="localization_input_node",
        output="screen",
        parameters=[
            str(localization_share / "config" / "localization_inputs.yaml"),
            {"use_sim_time": True},
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(localization_share / "config" / "ekf.yaml"), {"use_sim_time": True}],
    )

    return LaunchDescription([localization_input, ekf_node])
