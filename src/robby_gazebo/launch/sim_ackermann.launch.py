from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("robby_description"))
    control_share = Path(get_package_share_directory("robby_control"))
    gazebo_share = Path(get_package_share_directory("robby_gazebo"))
    localization_share = Path(get_package_share_directory("robby_localization"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    xacro_path = description_share / "urdf" / "Robby_v1.gazebo.xacro"
    mesh_dir = description_share / "meshes"
    controllers_file = gazebo_share / "config" / "ros2_controllers.yaml"
    world_file = gazebo_share / "worlds" / "empty.world.sdf"
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")

    robot_description = Command(
        [
            "xacro ",
            str(xacro_path),
            " mesh_dir:=",
            str(mesh_dir),
            " controllers_file:=",
            str(controllers_file),
            " use_lidar:=",
            use_lidar,
            " use_camera:=",
            use_camera,
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robby",
        output="screen",
        parameters=[
            {
                "topic": "/robot_description",
                "name": "robby",
                "allow_renaming": False,
                "x": 0.0,
                "y": 0.0,
                "z": 0.2,
                "R": 0.0,
                "P": 0.0,
                "Y": 0.0,
            }
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    steering_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    wheel_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    ackermann_cmd_node = Node(
        package="robby_control",
        executable="ackermann_cmd_node",
        name="ackermann_cmd_node",
        output="screen",
        parameters=[str(control_share / "config" / "ackermann_cmd.yaml"), {"use_sim_time": True}],
    )

    ackermann_controller_bridge = Node(
        package="robby_control",
        executable="ackermann_controller_bridge",
        name="ackermann_controller_bridge",
        output="screen",
        parameters=[
            str(control_share / "config" / "ackermann_controller_bridge.yaml"),
            {"use_sim_time": True},
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localization_share / "launch" / "localization.launch.py"))
    )
    evaluation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(localization_share / "launch" / "evaluation.launch.py"))
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )
    load_steering_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[steering_position_controller])
    )
    load_wheel_controller = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[wheel_velocity_controller])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_lidar",
                default_value="false",
                description="Enable the modular 2D lidar overlay and bridge /scan.",
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Enable the modular camera overlay and bridge /camera/image_raw.",
            ),
            gazebo,
            sim_bridge,
            robot_state_publisher,
            spawn_robot,
            load_joint_state_broadcaster,
            load_steering_controller,
            load_wheel_controller,
            ackermann_cmd_node,
            ackermann_controller_bridge,
            localization,
            evaluation,
        ]
    )
