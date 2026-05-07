# ITQ Robby Workspace

This workspace currently contains the first stage of the robot stack: the robot description, visualization setup, and the first cleanup needed to make the exported model usable in ROS 2.

The long-term goal is to turn this into a digital twin for a 4-wheel-drive, 4-wheel-steer robot with holonomic-style motion through coordinated steering and wheel control.

## Current Workspace Layout

```text
src/
├── README.md
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
- The URDF exported from SolidWorks and cleaned up for ROS 2 use
- A xacro source file that now acts as the main editable robot description
- A separate Gazebo xacro overlay so sim and `ros2_control` stay out of the base robot file
- The STL meshes used by RViz
- A ROS 2 launch file for visualization
- Basic package install rules so the URDF, meshes, and launch files are available after `colcon build`

Main files:
- `robby_description/urdf/Robby_v1.urdf`
- `robby_description/urdf/Robby_v1.core.xacro`
- `robby_description/urdf/Robby_v1.urdf.xacro`
- `robby_description/urdf/Robby_v1.gazebo.xacro`
- `robby_description/launch/display.launch.py`
- `robby_description/CMakeLists.txt`
- `robby_description/package.xml`

### `robby_control`

This package now contains the first control-side node for the digital twin.

It currently contains:
- A custom `swerve_odometry_node`
- A simple `ackermann_cmd_node` for car-like `/cmd_vel` testing
- A small bridge that turns Ackermann joint commands into live `/joint_states` for RViz testing
- A launch file to run the odometry node
- Launch files for odometry, Ackermann command generation, and Ackermann RViz visualization
- YAML config files for wheel joint names, steer joint names, wheel positions, wheel radius, and Ackermann parameters

Main files:
- `robby_control/robby_control/swerve_odometry_node.py`
- `robby_control/robby_control/ackermann_cmd_node.py`
- `robby_control/robby_control/ackermann_joint_state_bridge.py`
- `robby_control/launch/ackermann.launch.py`
- `robby_control/launch/ackermann_visualization.launch.py`
- `robby_control/launch/odometry.launch.py`
- `robby_control/config/ackermann_cmd.yaml`
- `robby_control/config/ackermann_joint_state_bridge.yaml`
- `robby_control/config/swerve_odometry.yaml`

### `robby_gazebo`

This package brings the robot into Gazebo Sim using `ros_gz_sim` and `gz_ros2_control`.

It currently contains:
- A minimal world file
- A `ros2_control` controller config for steering and wheel joints
- An Ackermann-focused simulation launch

Main files:
- `robby_gazebo/launch/sim_ackermann.launch.py`
- `robby_gazebo/config/ros2_controllers.yaml`
- `robby_gazebo/worlds/empty.world.sdf`

### `robby_localization`

This package contains the first localization stack for simulation and later hardware reuse.

It currently contains:
- An IMU preprocessor that fixes the IMU frame and fills in covariances when needed
- A `robot_localization` EKF config that fuses wheel odometry with IMU data
- A launch file for the sensor-fusion pipeline

Main files:
- `robby_localization/robby_localization/imu_preprocessor_node.py`
- `robby_localization/config/ekf.yaml`
- `robby_localization/launch/localization.launch.py`

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
- Collision geometry now uses simple boxes and cylinders instead of mesh collisions
- Rear steering is now locked to zero for an Ackermann-style simplification path
- A simple Ackermann `/cmd_vel` to joint-command node now exists
- A bridge now exists to visualize Ackermann commands directly in RViz without manual sliders
- Gazebo Sim bringup files now exist so the next phase can run through `ros2_control`
- A simulated IMU now exists in Gazebo
- The Gazebo launch now bridges IMU data into ROS
- An EKF now fuses `/wheel/odom` and `/imu/data/filtered`

Recommended launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_description display.launch.py
```

Recommended odometry launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_control odometry.launch.py
```

Recommended Ackermann command launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_control ackermann.launch.py
```

Recommended full Ackermann RViz test launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_control ackermann_visualization.launch.py
```

Recommended Gazebo Ackermann test launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_gazebo sim_ackermann.launch.py
```

Recommended localization-only launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/install/setup.bash
ros2 launch robby_localization localization.launch.py
```

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

The new xacro flow is a little better than the old hardcoded URDF because launch now injects the installed package mesh directory into the robot description at runtime.

This works for this machine and the installed workspace layout, but it is still not the final portable solution.

## Robot Model Notes

The robot is being treated as:
- 4 driven wheels
- 4 steering joints
- A future swerve or 4-wheel-steering platform

This means:
- `diff_drive_controller` is not the right controller
- A simple Ackermann setup is also not enough if all 4 wheels steer independently
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

### In Progress

- [ ] Turn the current description package into a clean digital twin base
- [ ] Decide the final package structure for control, localization, bringup, and simulation

### Next TODOs

- [ ] Create `robby_bringup` package
- [ ] Verify the Gazebo bringup end-to-end and tune joint/controller settings
- [ ] Tune EKF covariances and IMU noise against the simulated robot motion
- [ ] Add more simulated sensors beyond wheel odom and IMU
- [ ] Test holonomic sideways motion in simulation

### Later TODOs

- [ ] Replace absolute mesh paths with portable `package://` or xacro-based paths
- [ ] Add controllers for real hardware
- [ ] Add calibration workflow for wheel radius and steering zero offsets
- [ ] Add navigation stack integration if needed
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

1. Verify the Gazebo bringup end-to-end with the new IMU and EKF stack
2. Tune the EKF and IMU settings while driving the robot in simulation
3. Create `robby_gazebo`
4. Feed the odometry node from simulated joint states

That gives us a clean path from model -> odometry -> localization -> digital twin.
