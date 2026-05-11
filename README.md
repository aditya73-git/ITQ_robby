# ITQ Robby Workspace

ROS 2 Jazzy workspace for the ITQ Robby 4-wheel-drive, 4-wheel-steer robot.

This repository is centered on simulation-first development:
- swerve drive control
- wheel odometry + IMU EKF
- optional laser odometry
- `slam_toolbox` mapping
- saved-map localization with `map_server` + `amcl`
- Nav2 on top of the existing frame chain
- debug logging and offline plotting

## Repository Layout

- `src/robby_description`
  Robot xacro, meshes, RViz configs, and description launch files.
- `src/robby_control`
  Swerve kinematic model, controller, command node, and joint bridge.
- `src/robby_gazebo`
  Gazebo bringup, sim launch files, worlds, and sim defaults.
- `src/robby_localization`
  Wheel odom, IMU preprocessing, laser odom hookup, EKF, SLAM, AMCL, and Nav2 launch files.
- `src/robby_debug`
  CSV logging and plotting tools.
- `maps/`
  Saved maps used by localization.

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / ROS-Gazebo packages for Jazzy
- `python3-colcon-common-extensions`
- `python3-rosdep`

If `rosdep` has not been initialized on your machine yet:

```bash
sudo rosdep init
rosdep update
```

## Clone And Build

Clone the repository wherever you want your workspace to live:

```bash
git clone <your-repo-url> ITQ_robby
cd ITQ_robby
```

Install package dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

For later terminals:

```bash
cd /path/to/ITQ_robby
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Main Launch Files

- Sim launch: `robby_gazebo/launch/sim_swerve.launch.py`
- Sim defaults: `robby_gazebo/config/sim_swerve.yaml`
- Localization launch: `robby_localization/launch/localization.launch.py`
- Localization config: `robby_localization/config/localization.yaml`
- Nav2 config: `robby_localization/config/nav2.yaml`

## Bringup Modes

Default sim:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py
```

Mapping mode:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py enable_slam:=true enable_laser_odometry:=false
```

Saved-map localization:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py enable_slam:=false enable_saved_map_localization:=true enable_laser_odometry:=true
```

Saved-map localization + Nav2:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py enable_slam:=false enable_saved_map_localization:=true enable_nav2:=true enable_laser_odometry:=true
```

Override the saved map:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py enable_slam:=false enable_saved_map_localization:=true map_yaml_file:=/absolute/path/to/map.yaml
```

These defaults are controlled in `src/robby_gazebo/config/sim_swerve.yaml`.

## RViz

Robot model view:

```bash
ros2 launch robby_description display.launch.py
```

RViz for saved-map localization and Nav2:

```bash
ros2 launch robby_localization localization_rviz.launch.py
```

That RViz config is set up for:
- `/map`
- robot model
- TF
- `/scan`
- `/odometry/filtered`
- `2D Pose Estimate`
- `2D Goal Pose`

## Logging And Debugging

Launch sim with terminal output mirrored into a log file:

```bash
ros2 run robby_gazebo sim_with_logging
```

The logging behavior is controlled in `src/robby_gazebo/config/sim_swerve.yaml`:
- `enable_runtime_logging`
- `runtime_log_subdir`

Start the CSV debug logger:

```bash
ros2 launch robby_debug debug_tools.launch.py
```

Plot the latest debug log:

```bash
ros2 run robby_debug plot_debug_csv debug_logs
```

## Runtime Architecture

- `swerve_cmd_node` converts `/cmd_vel` into steering and wheel commands.
- `joint_command_bridge` forwards those commands to `ros2_control`.
- Gazebo publishes `/joint_states`, `/imu/data`, `/scan`, and `/ground_truth/odom`.
- `localization_input_node` publishes `/imu/data/filtered` and `/wheel/odom`.
- optional `laser_scan_matcher` publishes `/laser/odom`.
- EKF publishes `/odometry/filtered` and `odom -> base_footprint`.
- optional `map_server` publishes `/map`.
- optional `amcl` publishes `map -> odom`.
- optional `slam_toolbox` publishes `/map` and `map -> odom`.
- optional Nav2 publishes plans, recovery actions, and `/cmd_vel`.

Frame chain:

```text
map -> odom -> base_footprint -> base_link -> chassis/sensors
```

## Notes

- `base_footprint` is the ground-contact frame used by EKF, AMCL, SLAM, and Nav2.
- Sim defaults live in `src/robby_gazebo/config/sim_swerve.yaml`.
- Localization defaults live in `src/robby_localization/config/localization.yaml`.
- Nav2 tuning lives in `src/robby_localization/config/nav2.yaml`.
- The saved map packaged by default is `maps/arena_map.yaml`.

## Known Limitations

- Some mesh URIs still use absolute `file://` paths and may need cleanup on another machine.
- Wheel odometry is useful locally, but it is still being tuned against ground truth and laser-based localization.
- Nav2 controller behavior is still under active tuning for this swerve platform.
