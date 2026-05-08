# ITQ Robby Workspace

This workspace currently contains the first stage of the robot stack: the robot description, visualization setup, and the first cleanup needed to make the exported model usable in ROS 2.

The long-term goal is to turn this into a digital twin for a 4-wheel-drive, 4-wheel-steer robot with holonomic-style motion through coordinated steering and wheel control.

## Current Workspace Layout

```text
src/
├── README.md
├── robby_debug/
├── robby_control/
├── robby_gazebo/
├── robby_localization/
└── robby_description/
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    ├── meshes/
    ├── textures/
    ├── urdf/
    ├── include/
    └── src/
```

## Current Architecture

### `robby_description`

This package is the current source of truth for the robot model.

It contains:
- The cleaned ROS 2 xacro description that now acts as the source of truth for the robot model
- A separate Gazebo xacro overlay so sim and `ros2_control` stay out of the base robot file
- Reusable modular xacro overlays for lidar and camera sensors
- The STL meshes used by RViz
- A ROS 2 launch file for visualization
- Basic package install rules so the URDF, meshes, and launch files are available after `colcon build`

Legacy notes:
- The old SolidWorks-exported `Robby_v1.urdf`, ROS 1 XML launch files, and exporter CSV were archived outside the workspace at `/home/aditya/ros2_ws/ITQ_robby_archived/2026-05-08_legacy_description_files/`

Main files:
- `robby_description/urdf/Robby_v1.core.xacro`
- `robby_description/urdf/Robby_v1.urdf.xacro`
- `robby_description/urdf/Robby_v1.gazebo.xacro`
- `robby_description/urdf/Robby_v1.lidar.xacro`
- `robby_description/urdf/Robby_v1.camera.xacro`
- `robby_description/launch/display.launch.py`
- `robby_description/CMakeLists.txt`
- `robby_description/package.xml`

### `robby_control`

This package now contains the first control-side node for the digital twin.

It currently contains:
- A reusable 4WIS4WID kinematic model from the Lee and Li paper
- A paper-based non-linear kinematic controller
- A full `swerve_cmd_node` for independent 4-wheel steering and drive commands
- A joint-command bridge that forwards controller commands into `ros2_control`
- A launch file for the swerve command path
- YAML config files for the swerve controller and controller bridge

Legacy notes:
- The older Ackermann test stack and standalone `swerve_odometry_node` path were archived outside the workspace to keep the active stack focused.

Main files:
- `robby_control/src/robby_control/kinematic_model.py`
- `robby_control/src/robby_control/controller.py`
- `robby_control/src/robby_control/swerve_cmd_node.py`
- `robby_control/src/robby_control/joint_command_bridge.py`
- `robby_control/launch/swerve.launch.py`
- `robby_control/config/swerve_cmd.yaml`
- `robby_control/config/joint_command_bridge.yaml`

### `robby_gazebo`

This package brings the robot into Gazebo Sim using `ros_gz_sim` and `gz_ros2_control`.

It currently contains:
- A minimal world file
- A `ros2_control` controller config for steering and wheel joints
- A full swerve simulation launch
- A swerve mapping launch with lidar and `slam_toolbox`

Legacy notes:
- The older Ackermann simulation launch was archived outside the workspace.

Main files:
- `robby_gazebo/launch/sim_swerve.launch.py`
- `robby_gazebo/launch/sim_swerve_mapping.launch.py`
- `robby_gazebo/config/ros2_controllers.yaml`
- `robby_gazebo/worlds/empty.world.sdf`

### `robby_localization`

This package contains the first localization stack for simulation and later hardware reuse.

It currently contains:
- A localization input node that preprocesses IMU data and computes wheel odometry from joint states using the paper-based kinematic model
- A `robot_localization` EKF config that fuses wheel odometry with IMU data
- A state error monitor that compares a reference odometry source against the filtered estimate
- A launch file for the sensor-fusion pipeline
- A `slam_toolbox` mapping config and launch

Main files:
- `robby_localization/src/robby_localization/localization_input_node.py`
- `robby_localization/src/robby_localization/state_error_monitor_node.py`
- `robby_localization/config/ekf.yaml`
- `robby_localization/config/slam_toolbox.yaml`
- `robby_localization/launch/evaluation.launch.py`
- `robby_localization/launch/localization.launch.py`
- `robby_localization/launch/slam_mapping.launch.py`

### `robby_debug`

This package is the workspace toolbox for logging, plotting, and one-off debugging scripts.

It currently contains:
- A live CSV logger for ground truth, wheel odometry, EKF output, command input, and estimation error
- A simple offline plotting script for trajectories and error curves
- A small launch file to start the logger

Main files:
- `robby_debug/src/robby_debug/debug_log_node.py`
- `robby_debug/src/robby_debug/plot_debug_csv.py`
- `robby_debug/launch/debug_tools.launch.py`

## What Works Right Now

- The robot model loads in RViz
- The meshes are visible
- The steering joint names and mesh names were cleaned up
- The steering joints can now move in `joint_state_publisher_gui`
- The steering range is set to `+-90 deg` for testing
- A ROS 2 Python launch file is available for display
- The robot description can now be generated from xacro
- Wheelbase, track width, wheel radius, and steering geometry constants now live in xacro properties
- The first custom swerve odometry node is implemented and builds cleanly
- The wheel odometry path now supports calibrated effective wheel radius and effective rear track width
- Collision geometry now uses simple boxes and cylinders instead of mesh collisions
- A `base_footprint` ground-contact frame now exists below `base_link`
- A simple Ackermann `/cmd_vel` to joint-command node now exists
- A full swerve `/cmd_vel` to 8-joint command node now exists
- The paper-based kinematic model and controller are implemented in `robby_control`
- A bridge now exists to visualize Ackermann commands directly in RViz without manual sliders
- Gazebo Sim bringup files now exist so the next phase can run through `ros2_control`
- A simulated IMU now exists in Gazebo
- Optional modular 2D lidar and camera sensors now exist in the robot description and Gazebo overlay
- The Gazebo launch now bridges IMU data into ROS
- An EKF now fuses `/wheel/odom` and `/imu/data/filtered`
- Gazebo now publishes `/ground_truth/odom` as a reference source for evaluation
- A monitor now compares `/ground_truth/odom` against `/odometry/filtered`
- `slam_toolbox` now produces a 2D map from `/scan`

Recommended launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_description display.launch.py
```

Recommended Gazebo swerve test launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_gazebo sim_swerve.launch.py
```

Recommended Gazebo swerve mapping launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_gazebo sim_swerve_mapping.launch.py
```

Recommended localization-only launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_localization localization.launch.py
```

Recommended SLAM mapping-only launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_localization slam_mapping.launch.py
```

Recommended debug logger launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_debug debug_tools.launch.py
```

Recommended offline plot command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 run robby_debug plot_debug_csv /home/aditya/ros2_ws/ITQ_robby/debug_logs/<log_name>.csv
```

Reference evaluation is started automatically by the Gazebo sim launch. It compares:
- `/ground_truth/odom`
- `/odometry/filtered`

and publishes:
- `/state_estimation/error`

## Important Notes

### SolidWorks export caveat

The original SolidWorks export was not fully ROS 2 ready.

Problems we already fixed:
- Package URI naming did not match the real package name
- Several steer-related names had typos
- Meshes and URDF were not being installed by CMake
- The old XML launch files were ROS 1 style and not ideal for ROS 2 bringup
- The original mesh collisions were too detailed for a clean Gazebo collision model

### Current mesh path workaround

The URDF currently uses absolute `file://` mesh paths instead of `package://` mesh paths.

This was done because RViz was still failing to resolve the package mesh resources in the active runtime session even after the package naming was corrected.

### Odometry calibration note

The robot model and the odometry model are related, but they are not exactly the same thing.

- `robby_description` keeps the physical geometry from CAD and xacro
- `robby_localization/config/localization_inputs.yaml` now keeps the active wheel-odometry calibration values

For wheel odometry, the most important calibrated values right now are:
- `effective_wheel_radius`
- `effective_rear_track_width`

These are the values the odometry node should use to match measured robot motion, even if they differ slightly from the CAD dimensions. This is normal in both simulation tuning and real hardware bringup.

The new xacro flow is a little better than the old hardcoded URDF because launch now injects the installed package mesh directory into the robot description at runtime.

This works for this machine and the installed workspace layout, but it is still not the final portable solution.

## Robot Model Notes

The robot is being treated as:
- 4 driven wheels
- 4 steering joints
- A future swerve or 4-wheel-steering platform

This means:
- `diff_drive_controller` is not the right controller
- A simple Ackermann setup is only a temporary test mode
- We will likely need custom swerve-style kinematics, odometry, and command handling

## Suggested Future Package Split

This is the architecture we are aiming for next:

- `robby_description`
  - URDF, meshes, xacro, RViz config
- `robby_control`
  - custom swerve kinematics
  - `cmd_vel` to wheel speed and steering angle conversion
  - wheel and steer based odometry
  - future hardware or simulation interfaces
- `robby_localization`
  - IMU preprocessing
  - wheel odometry + IMU fusion
  - `robot_localization` EKF setup
- `robby_gazebo`
  - Gazebo world
  - robot spawn
  - simulated sensors and plugins
  - `gz_ros2_control` integration
- `robby_bringup`
  - full system launch files
  - sim bringup
  - real robot bringup later
- `robby_debug`
  - CSV loggers
  - plotting scripts
  - quick diagnostics

## TODO Tracker

Use this section as the project tracker. Check items when done, leave notes beside them if needed.

### Done

- [x] Import robot model into ROS 2 workspace
- [x] Clean mesh and steer naming typos in the URDF
- [x] Fix missing RViz mesh loading issue
- [x] Add install rules for `urdf/`, `meshes/`, and `launch/`
- [x] Add a working ROS 2 display launch file
- [x] Unlock steering joints and set steering range to `+-90 deg`
- [x] Verify the robot is visible in RViz
- [x] Verify wheel and steer joints can be moved from `joint_state_publisher_gui`
- [x] Convert the main robot description flow to xacro
- [x] Define wheel radius, track width, wheelbase, and steering geometry in one place
- [x] Create `robby_control` package
- [x] Implement the first custom wheel odometry node from wheel encoder and steer angle data
- [x] Lock rear steering joints to zero for the Ackermann-style simplification path
- [x] Create a first Ackermann `/cmd_vel` to joint-command node
- [x] Add a bridge from Ackermann joint commands to `/joint_states` for RViz-only testing
- [x] Create `robby_gazebo` package
- [x] Add first Gazebo Sim bringup with `gz_ros2_control`
- [x] Add a simulated IMU to Gazebo
- [x] Create `robby_localization` package
- [x] Add an EKF stack that fuses wheel odometry and IMU data
- [x] Add a Gazebo ground-truth reference odometry topic
- [x] Create `robby_debug` package
- [x] Add CSV logging and offline plotting tools
- [x] Add a state estimation error monitor for sim evaluation
- [x] Implement the paper-based 4WIS4WID kinematic model
- [x] Implement the paper-based outer-loop kinematic controller
- [x] Add a full 4WIS4WID command node and sim launch
- [x] Unlock rear steering again for full 4-wheel-steering operation
- [x] Add modular lidar and camera xacro overlays
- [x] Add `slam_toolbox` mapping from `/scan`
- [x] Add `base_footprint` as the ground-contact frame

### In Progress

- [ ] Turn the current description package into a clean digital twin base
- [ ] Decide the final package structure for control, localization, bringup, and simulation

### Next TODOs

- [ ] Create `robby_bringup` package
- [ ] Verify the Gazebo bringup end-to-end and tune joint/controller settings
- [ ] Calibrate `effective_wheel_radius` and `effective_rear_track_width` against `/ground_truth/odom`
- [ ] Tune EKF covariances and IMU noise against the simulated robot motion
- [ ] Add Nav2 on top of the current map, odom, and control stack
- [ ] Decide the future absolute reference path: GNSS, SLAM, landmarks, or multiple modes
- [ ] Test holonomic sideways motion and pure lateral control in simulation

### Later TODOs

- [ ] Replace absolute mesh paths with portable `package://` or xacro-based paths
- [ ] Add controllers for real hardware
- [ ] Add calibration workflow for per-wheel radius and steering zero offsets
- [ ] Add navigation stack integration if needed
- [ ] Add 3D sensing or depth fusion if 2D lidar is not sufficient for the target environment
- [ ] Add logging, diagnostics, and replay support
- [ ] Add documentation for real robot bringup

## Open Technical Questions

- What is the final steering strategy: true swerve, crab steering, or multiple drive modes?
- What are the exact wheel radius, wheelbase, and track width values?
- Will the real robot publish raw encoder counts, joint states, or both?
- What IMU hardware and frame convention will be used on the physical robot?
- Do we want Gazebo only, or a higher-fidelity simulator later as well?

## Recommended Next Step

The best next implementation step is:

1. Build a simple obstacle world so 2D lidar mapping has meaningful structure
2. Verify `slam_toolbox` map creation and save a first map
3. Add Nav2 using the current `map -> odom -> base_footprint` frame chain
4. Connect Nav2 `/cmd_vel` output to the 4WIS4WID command path

That gives us a clean path from model -> odometry -> localization -> mapping -> navigation.
