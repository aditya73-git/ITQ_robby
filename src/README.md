# ITQ Robby Workspace

Simulation-first ROS 2 workspace for the Robby 4-wheel-drive, 4-wheel-steer robot.

Current focus:
- swerve control
- EKF-based local state estimation
- LiDAR-based mapping with `slam_toolbox`
- saved-map localization with `nav2_map_server` + `amcl`
- Nav2 bringup on top of the existing frame chain
- debug logging and offline analysis

## Packages

- `robby_description`
  - robot xacro, meshes, RViz config, Gazebo sensor overlays
- `robby_control`
  - swerve kinematic model, controller, command node, joint bridge
- `robby_gazebo`
  - Gazebo launch, worlds, `ros2_control` config, sim defaults
- `robby_localization`
  - wheel odom, IMU preprocessing, LiDAR odom, EKF, optional SLAM
- `robby_debug`
  - CSV logging and plotting

## Active Entrypoints

- Sim launch:
  - `robby_gazebo/launch/sim_swerve.launch.py`
- Sim defaults:
  - `robby_gazebo/config/sim_swerve.yaml`
- Localization launch:
  - `robby_localization/launch/localization.launch.py`
- Localization config:
  - `robby_localization/config/localization.yaml`

Archived legacy files were moved outside the workspace to:
- `/home/aditya/ros2_ws/ITQ_robby_archived/`

## Current Architecture

- `swerve_cmd_node` converts `/cmd_vel` into steering and wheel commands
- `joint_command_bridge` forwards those commands to `ros2_control`
- Gazebo sim publishes:
  - `/joint_states`
  - `/imu/data`
  - `/scan`
  - `/ground_truth/odom`
- `localization_input_node` publishes:
  - `/imu/data/filtered`
  - `/wheel/odom`
- optional `laser_scan_matcher` publishes:
  - `/laser/odom`
- EKF publishes:
  - `/odometry/filtered`
  - `odom -> base_footprint`
- optional `nav2_map_server` publishes:
  - `/map`
- optional `amcl` publishes:
  - `map -> odom`
- optional `slam_toolbox` publishes:
  - `/map`
  - `map -> odom`
- optional Nav2 publishes:
  - global plans, local plans, recoveries, and `/cmd_vel`

Frame chain:
- `map -> odom -> base_footprint -> base_link -> chassis/sensors`

## Main Modes

- Local odometry mode
  - `enable_laser_odometry: true`
  - `enable_slam: false`
  - better local EKF output

- Mapping mode
  - `enable_laser_odometry: false`
  - `enable_slam: true`
  - more stable `slam_toolbox` behavior

- Saved-map localization mode
  - `enable_saved_map_localization: true`
  - `enable_slam: false`
  - `map_server` + `amcl` provide `map -> odom`

- Saved-map navigation mode
  - `enable_saved_map_localization: true`
  - `enable_nav2: true`
  - Nav2 runs on top of `map -> odom -> base_footprint`

These defaults are controlled in:
- `robby_gazebo/config/sim_swerve.yaml`

## Commands

Build:

```bash
cd /home/aditya/ros2_ws/ITQ_robby
source /opt/ros/jazzy/setup.bash
colcon build
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
```

RViz model view:

```bash
ros2 launch robby_description display.launch.py
```

Normal sim:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py
```

Normal sim with terminal logging captured to a file:

```bash
ros2 run robby_gazebo sim_with_logging
```

Logging control in `robby_gazebo/config/sim_swerve.yaml`:
- `enable_runtime_logging`
- `runtime_log_subdir`

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

RViz for saved-map localization and Nav2:

```bash
ros2 launch robby_localization localization_rviz.launch.py
```

Override the map file:

```bash
ros2 launch robby_gazebo sim_swerve.launch.py enable_slam:=false enable_saved_map_localization:=true map_yaml_file:=/absolute/path/to/map.yaml
```

Debug logger:

```bash
ros2 launch robby_debug debug_tools.launch.py
```

Plot latest log:

```bash
ros2 run robby_debug plot_debug_csv /home/aditya/ros2_ws/ITQ_robby/debug_logs
```

## Notes

- The robot model source of truth is the xacro stack in `robby_description/urdf/`.
- `base_footprint` is the ground-contact frame used by EKF and SLAM.
- `sim_swerve.yaml` is the place to change sim defaults.
- `localization.yaml` is the place to change localization defaults.
- `nav2.yaml` is the place to change planner / controller / costmap defaults.
- Some meshes still use absolute `file://` paths; that is a known portability cleanup item.

## Short TODO

Done:
- [x] swerve command path
- [x] full Gazebo bringup
- [x] IMU, LiDAR, and camera sim overlays
- [x] wheel odom + IMU EKF
- [x] optional LiDAR odom in EKF
- [x] `slam_toolbox` mapping
- [x] single active sim launch + config
- [x] single active localization launch + config

Next:
- [ ] publish meaningful covariance in `/laser/odom`
- [ ] tune EKF and SLAM for the current simulated robot
- [x] save and reload a first map
- [x] add Nav2 on top of `map -> odom -> base_footprint`
- [ ] test autonomous exploration workflow

Later:
- [ ] add `robby_bringup`
- [ ] improve mesh path portability
- [ ] add real hardware controllers and calibration flow
- [ ] add an independent local odometry source if needed
