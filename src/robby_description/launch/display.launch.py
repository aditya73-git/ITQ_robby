from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = Path(get_package_share_directory("robby_description"))
    xacro_path = package_share / "urdf" / "Robby_v1.urdf.xacro"
    mesh_dir = package_share / "meshes"
    rviz_config = package_share / "rviz" / "display.rviz"
    robot_description = xacro.process_file(
        str(xacro_path),
        mappings={"mesh_dir": str(mesh_dir)},
    ).toxml()

    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
            ),
        ]
    )
