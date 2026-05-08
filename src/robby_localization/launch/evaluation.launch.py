from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    monitor = Node(
        package="robby_localization",
        executable="state_error_monitor_node",
        name="state_error_monitor_node",
        output="screen",
        parameters=[
            {
                "reference_topic": "/ground_truth/odom",
                "estimate_topic": "/odometry/filtered",
                "error_topic": "/state_estimation/error",
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription([monitor])
