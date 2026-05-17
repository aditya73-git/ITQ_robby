import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc    = get_package_share_directory('robot_description')
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_gazebo  = get_package_share_directory('robot_gazebo')
    pkg_ros_gz  = get_package_share_directory('ros_gz_sim')

    urdf_file  = os.path.join(pkg_desc,    'urdf',   'robot.urdf.xacro')
    world_file = os.path.join(pkg_gazebo,  'worlds', 'empty.sdf')
    joy_config = os.path.join(pkg_bringup, 'config', 'joy_teleop.yaml')

    # In digital-twin mode the Pi publishes /cmd_vel over the network — no local teleop needed.
    # Set with_teleop:=true to drive the sim from a locally connected controller instead.
    with_teleop_arg = DeclareLaunchArgument(
        'with_teleop', default_value='false',
        description='Start joy+teleop locally (use false when Pi is controlling over network)'
    )
    with_teleop = LaunchConfiguration('with_teleop')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=true']),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'interpack_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen',
    )

    # Bridge Gazebo clock to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    spawn_diff_drive = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        )]
    )

    # Optional local teleop (only needed when Pi is not connected)
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_config, {'use_sim_time': True}],
        condition=IfCondition(with_teleop),
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config, {'use_sim_time': True}],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        condition=IfCondition(with_teleop),
        output='screen',
    )

    return LaunchDescription([
        with_teleop_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        spawn_joint_state_broadcaster,
        spawn_diff_drive,
        joy,
        teleop,
    ])
