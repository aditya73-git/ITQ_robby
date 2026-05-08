from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = Path(get_package_share_directory("robby_gazebo"))
    localization_share = Path(get_package_share_directory("robby_localization"))

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "sim_4wis4wid.launch.py")),
        launch_arguments={
            "use_lidar": "true",
            "use_camera": "true",
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localization_share / "launch" / "slam_mapping.launch.py"))
    )

    return LaunchDescription([sim_launch, slam_launch])
