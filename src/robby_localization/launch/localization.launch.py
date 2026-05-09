from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    localization_share = Path(get_package_share_directory("robby_localization"))
    enable_laser_odometry = LaunchConfiguration("enable_laser_odometry")
    enable_slam = LaunchConfiguration("enable_slam")
    localization_params = str(localization_share / "config" / "localization.yaml")

    localization_input = Node(
        package="robby_localization",
        executable="localization_input_node",
        name="localization_input_node",
        output="screen",
        parameters=[localization_params],
    )

    laser_odometry = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_odometry_node",
        output="screen",
        condition=IfCondition(enable_laser_odometry),
        parameters=[localization_params],
    )

    ekf_with_laser = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(enable_laser_odometry),
        parameters=[localization_params],
    )

    ekf_without_laser = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=UnlessCondition(enable_laser_odometry),
        parameters=[
            localization_params,
            {
                "odom1": "",
                "odom1_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
            },
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        condition=IfCondition(enable_slam),
        parameters=[ParameterFile(localization_params, allow_substs=True)],
    )

    configure_slam = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
        condition=IfCondition(enable_slam),
    )

    activate_slam_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state="inactive",
            entities=[
                LifecycleTransition(
                    lifecycle_node_names=["/slam_toolbox"],
                    transition_ids=[Transition.TRANSITION_ACTIVATE],
                )
            ],
        ),
        condition=IfCondition(enable_slam),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_laser_odometry",
                default_value="true",
                description="Start laser odometry and fuse /laser/odom into the EKF.",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Start slam_toolbox mapping from /scan.",
            ),
            localization_input,
            laser_odometry,
            ekf_with_laser,
            ekf_without_laser,
            slam_toolbox_node,
            configure_slam,
            activate_slam_on_inactive,
        ]
    )
