# UR10e + Robotiq 2F-85 Robot Cell

Combined robot description package for UR10e robotic arm with Robotiq 2F-85 gripper.

## Features

- ✅ Complete URDF combining UR10e and Robotiq 2F-85
- ✅ Works with `ur_robot_driver` for real robot control
- ✅ Works with `ur_moveit_config` for motion planning
- ✅ Compatible with Gazebo/Ignition simulation
- ✅ Supports fake hardware for testing without robot
- ✅ ROS 2 Jazzy Jalisco (Ubuntu 24.04)

## Installation

### Quick Setup (Recommended)

```bash
# Create workspace
mkdir -p ~/ws_ur10e_robotiq/src
cd ~/ws_ur10e_robotiq/src

# Clone this repository
git clone https://github.com/YOUR_USERNAME/ur10e_robotiq_cell.git

# Import dependencies (UR packages, Robotiq gripper, serial)
vcs import < ur10e_robotiq_cell/ur10e_robotiq_cell.repos

# Install ROS dependencies
cd ~/ws_ur10e_robotiq
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Usage

### 1. Real Robot Control (UR Tool Communication)

**Recommended setup** - Gripper connected through UR robot's tool communication:

```bash
# Launch with UR tool communication (gripper connected to robot)
ros2 launch ur10e_robotiq_cell ur10e_robotiq_control.launch.py \
  robot_ip:=192.168.56.101
```

This uses the UR robot's built-in tool communication interface via TCP/socat to communicate with the Robotiq gripper.

**Parameters:**
- `robot_ip` - IP address of UR10e robot (default: 192.168.56.101)
- `launch_rviz` - Launch RViz visualization (default: true)
- `use_fake_hardware` - Use fake hardware (default: false)
- `use_tool_communication` - Enable UR tool communication (default: true)
- `tool_device_name` - Virtual serial device name (default: /tmp/ttyUR)
- `tool_tcp_port` - TCP port for tool communication (default: 54321)
- `gripper_com_port` - COM port for gripper (default: /tmp/ttyUR)

### 1b. Direct Serial Connection (Alternative)

If gripper is connected directly to PC via USB-to-RS485:

```bash
# Launch with direct serial connection
ros2 launch ur10e_robotiq_cell ur10e_robotiq_control.launch.py \
  robot_ip:=192.168.56.101 \
  use_tool_communication:=false \
  gripper_com_port:=/dev/ttyUSB0
```

### 2. MoveIt Motion Planning

```bash
# Launch MoveIt with combined robot
ros2 launch ur10e_robotiq_cell ur10e_robotiq_moveit.launch.py \
  launch_rviz:=true
```

**Parameters:**
- `launch_rviz` - Launch RViz with MoveIt (default: true)
- `use_fake_hardware` - Use fake hardware for testing (default: false)

### 3. Fake Hardware (Testing)

```bash
# Test with fake hardware (no real robot needed)
ros2 launch ur10e_robotiq_cell ur10e_robotiq_control.launch.py \
  use_fake_hardware:=true \
  launch_rviz:=true
```

### 4. Gazebo Simulation

```bash
# Coming soon - Gazebo support
```

## Gripper Mounting

The gripper is attached to the UR10e `tool0` frame. If your gripper is mounted with a different offset or rotation, edit:

```
urdf/ur10e_robotiq.urdf.xacro
```

And adjust the transform at line ~80:

```xml
<origin xyz="0 0 0" rpy="0 0 0"/>
```

## Dependencies

- `ur_description` - UR robot descriptions
- `ur_robot_driver` - UR robot driver
- `ur_moveit_config` - MoveIt configuration for UR robots
- `robotiq_description` - Robotiq gripper URDF
- `robotiq_driver` - Robotiq gripper hardware interface (Jazzy port)
- `serial` - Serial communication library

All dependencies are automatically pulled using the `.repos` file.

## File Structure

```
ur10e_robotiq_cell/
├── CMakeLists.txt
├── package.xml
├── README.md
├── ur10e_robotiq_cell.repos        # Dependencies file
├── urdf/
│   └── ur10e_robotiq.urdf.xacro    # Combined robot description
├── launch/
│   ├── ur10e_robotiq_control.launch.py   # Real robot control
│   └── ur10e_robotiq_moveit.launch.py    # MoveIt planning
├── config/                         # (Future: MoveIt configs)
├── meshes/                         # (Future: custom meshes)
└── rviz/                           # (Future: RViz configs)
```

## Contributing

Issues and pull requests welcome!

## License

BSD-3-Clause

## Credits

- UR packages: [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- Robotiq Jazzy port: [bryceag11/ros2_robotiq_gripper](https://github.com/bryceag11/ros2_robotiq_gripper)
- Serial library: [tylerjw/serial](https://github.com/tylerjw/serial)
