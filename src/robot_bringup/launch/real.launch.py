import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc    = get_package_share_directory('robot_description')
    pkg_bringup = get_package_share_directory('robot_bringup')

    urdf_file  = os.path.join(pkg_desc,    'urdf',   'robot.urdf.xacro')
    ctrl_config = os.path.join(pkg_bringup, 'config', 'ros2_controllers.yaml')
    joy_config  = os.path.join(pkg_bringup, 'config', 'joy_teleop.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port connected to the ESP32'
    )
    serial_port = LaunchConfiguration('serial_port')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file,
                 ' use_sim:=false',
                 ' serial_port:=', serial_port]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ctrl_config,
        ],
        output='screen',
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    spawn_diff_drive = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        )]
    )

    js_publisher = Node(
        package='robot_bringup',
        executable='js_publisher',
        name='js_publisher',
        parameters=[{'dev': '/dev/input/js0', 'autorepeat_rate': 20.0}],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_diff_drive,
        js_publisher,
        teleop,
    ])
