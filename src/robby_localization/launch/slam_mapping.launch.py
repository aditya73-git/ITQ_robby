from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode, LifecycleTransition
from launch_ros.event_handlers import OnStateTransition
from launch_ros.descriptions import ParameterFile
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    localization_share = Path(get_package_share_directory("robby_localization"))
    slam_params_file = localization_share / "config" / "slam_toolbox.yaml"

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            ParameterFile(str(slam_params_file), allow_substs=True),
            {"use_sim_time": True},
        ],
    )

    configure_slam = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
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
        )
    )

    return LaunchDescription([slam_toolbox_node, activate_slam_on_inactive, configure_slam])
