# ROS2 SCARA Robot Project

A complete ROS2 implementation of a SCARA (Selective Compliance Assembly Robot Arm) robotic system with forward/inverse kinematics, motion planning, and gripper control.

## 📋 Overview

This project implements a 4-DOF SCARA robot with an integrated gripper in ROS2. The system includes:
- Complete URDF robot model with visual representation
- Robot driver with inverse kinematics solver
- Motion planning and waypoint-based trajectory execution
- Gripper control integration
- RViz visualization
- Joint state publishing and TF broadcasting

## 🏗️ System Architecture

### Packages

#### 1. `robot_bringup`
Main package for launching and configuring the robot system.
- **Launch Files**: `robot_system.launch.py` - Complete system launch configuration
- **URDF**: `scara_robot.urdf` - Robot description with 4 joints + gripper
- **Config**: `waypoints.json` - Predefined motion sequences
- **RViz**: Configuration files for visualization

#### 2. `robot_controller`
Core control package implementing robot kinematics and motion control.
- **robot_driver**: IK solver, joint interpolation, and state publishing
- **motion_publisher_node**: Waypoint-based mission execution

## 🤖 Robot Specifications

### Joint Configuration
- **Shoulder Joint** (Revolute): ±3.14 rad, controls base rotation
- **Elbow Joint** (Revolute): ±3.14 rad, controls second arm segment
- **Quill Joint** (Prismatic): 0.0-0.5 m, vertical translation
- **Theta Joint** (Revolute): ±6.28 rad, end-effector rotation
- **Gripper**: Binary control (open/close) with smooth interpolation

### Link Dimensions
- **Link 1 (L1)**: 0.5 m
- **Link 2 (L2)**: 0.4 m
- **Base Height**: 0.3 m
- **Gripper Range**: 0.03 m maximum travel

### Motion Limits
- **Max Angular Velocity**: 1.0 rad/s
- **Max Linear Velocity**: 0.5 m/s (prismatic joint)
- **Max Gripper Speed**: 0.05 m/s
- **Control Loop**: 50 ms (20 Hz)

## 🚀 Getting Started

### Prerequisites

- ROS2 (tested with Humble/Foxy)
- Ubuntu 20.04/22.04
- Dependencies:
  ```bash
  sudo apt install ros-$ROS_DISTRO-robot-state-publisher
  sudo apt install ros-$ROS_DISTRO-rviz2
  sudo apt install ros-$ROS_DISTRO-tf2-ros
  sudo apt install nlohmann-json3-dev
  ```

### Building the Project

1. Clone the repository:
   ```bash
   git clone https://github.com/becheanucornel/ROS2-Project---SCARA-Robot.git
   cd ROS2-Project---SCARA-Robot
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## 🎮 Running the System

### Launch Complete System

```bash
ros2 launch robot_bringup robot_system.launch.py
```

This will start:
- Robot driver node (IK solver and controller)
- Motion publisher (waypoint executor)
- Robot state publisher
- RViz2 visualization
- TF2 static transform broadcaster

### Individual Nodes

**Robot Driver Only:**
```bash
ros2 run robot_controller robot_driver
```

**Motion Publisher Only:**
```bash
ros2 run robot_controller motion_publisher_node --ros-args -p waypoint_file:=/path/to/waypoints.json
```

## 📡 ROS2 Topics

### Published Topics
- `/joint_states` (sensor_msgs/JointState) - Current joint positions
- `/tf` (tf2_msgs/TFMessage) - Transform tree

### Subscribed Topics
- `/scara_commands` (geometry_msgs/PoseStamped) - Target end-effector poses with gripper commands

## 🎯 Waypoint System

### Waypoint Format (JSON)

```json
{
  "id": 1,
  "x": 0.9,
  "y": 0.0,
  "z": 0.2,
  "yaw": 0.0,
  "gripper": 0.0,
  "note": "Description"
}
```

**Fields:**
- `x, y, z`: Cartesian coordinates (meters)
- `yaw`: End-effector orientation (radians)
- `gripper`: 0.0 = open, 1.0 = closed
- `note`: Human-readable description

### Example Mission Sequence

The default `waypoints.json` includes a pick-and-place demonstration:
1. Home position
2. Approach object
3. Descend to pick height
4. Close gripper
5. Lift object
6. Move to place location
7. Descend to place height
8. Release object
9. Return to home

## 🔧 Configuration

### Updating Robot Dimensions

Edit link lengths in:
- `src/robot_bringup/urdf/scara_robot.urdf` (visual model)
- `src/robot_controller/src/robot_driver.cpp` (L1, L2 constants)

### Adjusting Motion Speed

Modify speed limits in `src/robot_controller/src/robot_driver.cpp`:
```cpp
const double MAX_SPEED_RAD = 1.0;      // Joint angular velocity
const double MAX_SPEED_LIN = 0.5;      // Prismatic joint speed
const double MAX_SPEED_GRIPPER = 0.05; // Gripper speed
```

### Waypoint Timing

Change interval in `src/robot_controller/src/motion_publisher.cpp`:
```cpp
timer_ = this->create_wall_timer(3000ms, ...); // 3 seconds between waypoints
```

## 🧮 Inverse Kinematics

The robot driver implements analytical IK for the SCARA configuration:

**Given:** Target position (x, y, z) and orientation (yaw)

**Computed:**
1. **Shoulder & Elbow angles** - Using geometric solution for 2-link planar arm
2. **Quill extension** - Direct mapping from z-coordinate
3. **Theta angle** - Compensation to maintain end-effector orientation

The solver validates workspace reachability and returns false for unreachable targets.

## 📊 Monitoring

### View Joint States
```bash
ros2 topic echo /joint_states
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
```

### Monitor Commands
```bash
ros2 topic echo /scara_commands
```

## 🐛 Troubleshooting

### Robot Not Moving
- Verify all nodes are running: `ros2 node list`
- Check if commands are being published: `ros2 topic hz /scara_commands`
- Ensure waypoint file path is correct in launch file

### IK Solver Warnings
- Target may be outside workspace (check x² + y² within [L1-L2]² to [L1+L2]²)
- Z-coordinate should be within quill joint limits (0.0-0.5m from base)

### RViz Not Showing Robot
- Verify robot_state_publisher is running
- Check that /joint_states topic is publishing
- Ensure TF frames are being broadcast

## 📝 Development Notes

### Gripper Command Encoding
The gripper state (0.0/1.0) is encoded in the `orientation.x` field of PoseStamped messages, as SCARA robots don't use this degree of freedom. The robot driver extracts and applies this value to control gripper closure.

### Control Loop
The system uses a 50ms timer for smooth interpolation between current and target joint states, ensuring controlled motion without abrupt changes.

## 👨‍💻 Author

**Becheanu Cornel**
- Email: becheanucornel28@gmail.com
- GitHub: [@becheanucornel](https://github.com/becheanucornel)

## 📄 License

TODO: License declaration

## 🙏 Acknowledgments

Built with ROS2, leveraging:
- `robot_state_publisher` for URDF processing
- `tf2` for coordinate transformations
- `nlohmann/json` for configuration parsing
- `RViz2` for visualization

---

**Project Status:** Active Development
**Version:** 0.0.0
**Last Updated:** December 2025
