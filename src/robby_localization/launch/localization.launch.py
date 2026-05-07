from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = Path(get_package_share_directory("robby_localization"))

    imu_preprocessor = Node(
        package="robby_localization",
        executable="imu_preprocessor_node",
        name="imu_preprocessor_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/imu/data",
                "output_topic": "/imu/data/filtered",
                "frame_id": "imu_link",
                "use_sim_time": True,
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(localization_share / "config" / "ekf.yaml"), {"use_sim_time": True}],
    )

    return LaunchDescription([imu_preprocessor, ekf_node])
