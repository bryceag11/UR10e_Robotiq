# UR10e + Robotiq 2F-85 MoveIt Integration for ROS 2 Jazzy

## Overview

This repository provides a complete integration of the UR10e robot arm with the Robotiq 2F-85 gripper for ROS 2 Jazzy Jalisco on Ubuntu 24.04. It includes:

1. Robotiq gripper driver ported to ROS 2 Jazzy
2. Combined URDF with calibrated TCP
3. MoveIt2 configuration for motion planning with arm, gripper, or both
4. Pre-configured collision avoidance between arm and gripper

## Features

- Full MoveIt2 integration with three planning groups: `ur_manipulator`, `gripper`, and `ur10e_robotiq`
- ROS2 gripper controller
- Calibrated TCP at gripper tip for accurate motion planning
- Collision checking configuration
- RViz visualization with real-time gripper state

## Prerequisites

### System Requirements

- Ubuntu 24.04 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco

### Required ROS 2 Packages

Install ROS 2 Jazzy following the official installation guide: https://docs.ros.org/en/jazzy/Installation.html

Install required dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  python3-colcon-common-extensions \
  python3-vcstool \
  git
```

### Hardware Requirements

- Universal Robots UR10e robot arm
- Robotiq 2F-85 gripper

## Installation

### 1. Create workspace

```bash
mkdir -p ~/ws_robotiq/src
cd ~/ws_robotiq/src
```

### 2. Clone this repository

```bash
git clone https://github.com/bryceag11/UR10e_Robotiq
```

### 3. Import dependencies

```bash
cd ~/ws_robotiq
vcs import src < ur_robotiq_jazzy.repos
```

This imports:
- [serial](https://github.com/tylerjw/serial) - Serial communication library
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) - UR robot driver

### 4. Install rosdep dependencies

```bash
cd ~/ws_robotiq
rosdep update
rosdep install --from-paths src -y --ignore-src
```

### 5. Build workspace

```bash
cd ~/ws_robotiq
colcon build --symlink-install
```

### 6. Source workspace

```bash
source ~/ws_robotiq/install/setup.bash
```

Add to your `.bashrc` for automatic sourcing:

```bash
echo "source ~/ws_robotiq/install/setup.bash" >> ~/.bashrc
```

## Usage

### Launching the Complete System

The system requires two terminals for proper operation.

#### Terminal 1: Robot Driver and Gripper Controller

Launch the UR driver and gripper controller:

```bash
source ~/ws_robotiq/install/setup.bash
ros2 launch ur10e_robotiq_cell ur10e_robotiq_control.launch.py robot_ip:=192.168.56.101 launch_rviz:=false
```

This will:
- Start the UR robot driver
- Wait for External Control program connection
- Launch the URScript gripper controller after 5 seconds

On the robot teach pendant, press play on the External Control program. The robot will automatically connect to the ROS driver.

#### Terminal 2: MoveIt

After the robot driver is connected, launch MoveIt:

```bash
source ~/ws_robotiq/install/setup.bash
ros2 launch ur10e_robotiq_moveit_config ur_moveit.launch.py ur_type:=ur10e
```

This opens RViz with the MoveIt motion planning interface.

### Using MoveIt for Motion Planning

In the MoveIt RViz interface:

1. Select a planning group from the dropdown:
   - `ur_manipulator` - Control arm only
   - `gripper` - Control gripper only
   - `ur10e_robotiq` - Control arm and gripper together

2. Click "Update" next to "Start State" to sync with the real robot

3. Drag the interactive marker to set a goal pose or select a predefined pose

4. Click "Plan" to generate a trajectory

5. Click "Execute" to run the trajectory on the real robot

### Controlling the Gripper

#### Via MoveIt

1. Select the `gripper` planning group
2. Use the "open" or "close" predefined poses
3. Plan and execute

#### Via Action Client

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 50.0}}"
```

Position range: 0.0 (fully open) to 0.085 (fully closed) meters

#### Via ROS 2 Topic

Monitor gripper state:

```bash
ros2 topic echo /joint_states
```

## Package Descriptions

### ros2_robotiq_gripper

Robotiq gripper driver ported to ROS 2 Jazzy. Includes:
- Hardware interface for ros2_control integration
- Controller implementations
- URDF description for Robotiq 2F-85

Source: [PickNikRobotics/ros2_robotiq_gripper Jazzy Port](https://github.com/bryceag11/ros2_robotiq_gripper)

### ur10e_robotiq_cell

Main integration package containing:
- `urdf/` - Combined UR10e + Robotiq URDF with calibrated TCP
- `launch/` - Launch files for robot driver and description
- `scripts/` - URScript gripper controller
- `config/` - ros2_control controllers configuration
- `resources/` - URScript files with gripper functions

Key files:
- `urdf/ur10e_robotiq.urdf.xacro` - Complete robot description
- `launch/ur10e_robotiq_control.launch.py` - Main control launch file
- `scripts/urscript_gripper_controller.py` - Gripper action server

### ur10e_robotiq_moveit_config

MoveIt2 configuration package containing:
- `config/` - MoveIt configuration files
- `launch/` - MoveIt launch files
- `srdf/` - Semantic robot description with planning groups

Key configurations:
- Three planning groups: ur_manipulator, gripper, ur10e_robotiq
- TCP configured at calibrated gripper tip
- Collision checking between arm and gripper
- OMPL motion planning with multiple planners

## Architecture

### TCP Configuration

The TCP (Tool Center Point) is configured at the calibrated gripper tip:
- Position: X=-11.01mm, Y=-1.3mm, Z=232.63mm from gripper base
- Rotation: RX=0.0094rad, RY=0.0022rad, RZ=0.0397rad


### Gripper Joint Limits

The Robotiq 2F-85 gripper joint limits:
- Range: 0.0 to 0.804 radians (0 to 46 degrees)
- Stroke: 0 to 85mm
- Max velocity: 0.5 rad/s
- Max acceleration: 2.0 rad/s²

## Troubleshooting

### Robot driver fails to connect

Ensure:
1. Robot IP is correct (default: 192.168.56.101)
2. External Control URCap is installed on robot
3. External Control program is running on teach pendant
4. Network connection is stable

### MoveIt execution fails

Check:
1. Start state is updated (click "Update" in MoveIt)
2. Robot is not in protective stop
3. Gripper controller is running (check `ros2 action list`)
4. No collision errors in move_group logs

### Gripper not responding

Verify:
1. URScript gripper controller node is running
2. Robot is connected (External Control active)
3. Gripper is activated (run activation script if needed)
4. Action server is available: `ros2 action list | grep gripper_cmd`

### Planning succeeds but execution is rejected

Increase the start state tolerance in `moveit_controllers.yaml`:

```yaml
trajectory_execution:
  allowed_start_tolerance: 0.1  # Increase from 0.05
```

## Configuration Files

Key configuration files you may need to modify:

- `src/ur10e_robotiq_cell/urdf/ur10e_robotiq.urdf.xacro` - Robot description, TCP calibration
- `src/ur10e_robotiq_cell/config/ur10e_robotiq_controllers.yaml` - ros2_control controllers
- `src/ur10e_robotiq_moveit_config/config/moveit_controllers.yaml` - MoveIt controllers
- `src/ur10e_robotiq_moveit_config/config/joint_limits.yaml` - Joint velocity/acceleration limits
- `src/ur10e_robotiq_moveit_config/srdf/ur10e_robotiq.srdf.xacro` - Planning groups, collision rules

## Launch File Arguments

### ur10e_robotiq_control.launch.py

```bash
ros2 launch ur10e_robotiq_cell ur10e_robotiq_control.launch.py \
  robot_ip:=192.168.56.101 \
  launch_rviz:=false
```

Arguments:
- `robot_ip` - IP address of UR robot (default: 192.168.56.101)
- `launch_rviz` - Launch RViz with robot model (default: true)
- `use_fake_hardware` - Use mock hardware instead of real robot (default: false)

### ur_moveit.launch.py

```bash
ros2 launch ur10e_robotiq_moveit_config ur_moveit.launch.py \
  ur_type:=ur10e \
  launch_rviz:=true
```

Arguments:
- `ur_type` - UR robot model (required: ur10e)
- `launch_rviz` - Launch RViz with MoveIt interface (default: true)
- `launch_servo` - Enable MoveIt Servo for real-time control (default: false)

## Credits and References

This integration builds upon:

- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) - UR robot driver
- [ros2_robotiq_gripper](https://github.com/bryceag11/ros2_robotiq_gripper) - Robotiq gripper driver
- [serial](https://github.com/tylerjw/serial) - Serial communication library
- [Moveit2](https://moveit.picknik.ai/main/index.html) - Motion planning framework
