# ITQ Robby Workspace

This workspace currently contains the first stage of the robot stack: the robot description, visualization setup, and the first cleanup needed to make the exported model usable in ROS 2.

The long-term goal is to turn this into a digital twin for a 4-wheel-drive, 4-wheel-steer robot with holonomic-style motion through coordinated steering and wheel control.

## Current Workspace Layout

```text
src/
├── README.md
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
- The STL meshes used by RViz
- A ROS 2 launch file for visualization
- Basic package install rules so the URDF, meshes, and launch files are available after `colcon build`

Main files:
- `robby_description/urdf/Robby_v1.urdf`
- `robby_description/urdf/Robby_v1.urdf.xacro`
- `robby_description/launch/display.launch.py`
- `robby_description/CMakeLists.txt`
- `robby_description/package.xml`

## What Works Right Now

- The robot model loads in RViz
- The meshes are visible
- The steering joint names and mesh names were cleaned up
- The steering joints can now move in `joint_state_publisher_gui`
- The steering range is set to `+-90 deg` for testing
- A `world -> base_link` root was added so TF works better in RViz
- A ROS 2 Python launch file is available for display
- The robot description can now be generated from xacro
- Wheelbase, track width, wheel radius, and steering geometry constants now live in xacro properties

Recommended launch command:

```bash
source /home/aditya/ros2_ws/ITQ_robby/src/install/setup.bash
ros2 launch robby_description display.launch.py
```

## Important Notes

### SolidWorks export caveat

The original SolidWorks export was not fully ROS 2 ready.

Problems we already fixed:
- Package URI naming did not match the real package name
- Several steer-related names had typos
- Meshes and URDF were not being installed by CMake
- The old XML launch files were ROS 1 style and not ideal for ROS 2 bringup

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
  - future hardware or simulation interfaces
- `robby_localization`
  - wheel odometry
  - IMU fusion
  - `robot_localization` EKF setup
- `robby_gazebo`
  - Gazebo world
  - robot spawn
  - simulated sensors and plugins
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
- [x] Add `world -> base_link` transform in URDF
- [x] Unlock steering joints and set steering range to `+-90 deg`
- [x] Verify the robot is visible in RViz
- [x] Verify wheel and steer joints can be moved from `joint_state_publisher_gui`
- [x] Convert the main robot description flow to xacro
- [x] Define wheel radius, track width, wheelbase, and steering geometry in one place

### In Progress

- [ ] Turn the current description package into a clean digital twin base
- [ ] Decide the final package structure for control, localization, bringup, and simulation

### Next TODOs

- [ ] Add an RViz config file so the display comes up with a stable default setup
- [ ] Create `robby_control` package
- [ ] Create `robby_localization` package
- [ ] Create `robby_gazebo` package
- [ ] Create `robby_bringup` package
- [ ] Add `ros2_control` interfaces to the robot model
- [ ] Implement a custom swerve or 4-wheel-steer command node
- [ ] Implement a custom wheel odometry node from wheel encoder and steer angle data
- [ ] Add IMU topic integration
- [ ] Add `robot_localization` EKF for odom plus IMU fusion
- [ ] Spawn the robot in Gazebo
- [ ] Simulate IMU and wheel joint feedback in Gazebo
- [ ] Publish `/odom` and proper TF chain for the digital twin
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

1. Create `robby_control`
2. Implement the first custom `swerve_odometry_node`
3. Create `robby_localization`
4. Add IMU fusion using `robot_localization`

That gives us a clean path from model -> odometry -> localization -> digital twin.
