# Technical Project - Multi-Robot Coordination System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](https://classic.gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-1.0-green.svg)](https://navigation.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A ROS2-based multi-robot coordination system featuring autonomous navigation and manipulation tasks. The project implements a coordinated workflow between a mobile robot (Fra2mo) and two manipulator arms (Armando 1 & 2) in a simulated workshop environment.

![Project Demo](docs/images/demo.gif)

## 🎯 Project Overview

This project demonstrates advanced multi-robot coordination using:
- **Fra2mo**: Mobile robot with autonomous navigation capabilities
- **Armando 1 & 2**: 4-DOF manipulator arms for pick-and-place operations
- **Mission Coordinator**: State machine-based central controller
- **Nav2 Stack**: Advanced path planning and obstacle avoidance

### Workflow

1. **Initialization**: All robots move to home position and publish READY status
2. **Loading Phase**: Armando 1 picks a hammer and places it on Fra2mo
3. **Navigation Phase 1**: Fra2mo navigates to Waypoint 1
4. **Navigation Phase 2**: Fra2mo continues to Waypoint 2 and waits
5. **Unloading Phase**: Armando 2 retrieves the hammer from Fra2mo and stores it in a cabinet
6. **Return Phase**: Fra2mo completes remaining waypoints and returns home
7. **Mission Complete**: All robots return to home position

## 📋 Table of Contents

- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Usage](#usage)
- [Configuration](#configuration)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 💻 System Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble Hawksbill
- **Gazebo**: Classic 11
- **RAM**: Minimum 8GB (16GB recommended)
- **CPU**: Multi-core processor recommended
- **Storage**: ~5GB for workspace and dependencies

## 🚀 Installation

### 1. Install ROS2 Humble

```
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
# Install ROS2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-nav2-simple-commander

2. Clone and Build the Project
```
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/EndSword33/Technical-Project.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
###📁 Package Structure
```
Technical-Project/
├── ros2_fra2mo/                    # Main mobile robot package
│   ├── config/
│   │   ├── amcl.yaml         
│   │   ├── explore.yaml      
│   │   ├── navigation.yaml   
│   │   └── slam.yaml 
│   ├── launch/
│   │   ├── display_fra2mo.launch.py          
│   │   ├── fra2mo_amcl.launch.py          
│   │   ├── fra2mo_explore.launch.py         
│   │   ├── fra2mo_slam.launch.py          
│   │   ├── gazebo_fra2mo.launch.py          
│   │   ├── fra2mo_navigation.launch.py      
│   │   └── waypoint_follow.launch.py        
│   ├── meshes/                     
│   ├── scripts/
│   │   ├── follow_waypoints.py
│   │   ├── reach_goal.py
│   │   └── navigate.py
│   ├── urdf/
│   │   └── fra2mo.urdf.xacro       # Robot description
│   └── worlds/
│       └── workshop.sdf    # Gazebo world
│
├── final_project/                  # Armando manipulator package
│   ├── config/
│       └──rviz/
│   │      ├── rviz.rviz
│   │      └── standing.rviz    
│   │   ├── arm_params.yaml
│   │   ├── armando_controllers_1.yaml
│   │   └── armando_controllers_2.yaml
│   ├── launch/
│   │   └── __pycache__/
│   │   ├── coordination.launch.py 
│   │   └── spawn_armando.launch.py 
│   ├── meshes/
│   ├── scripts/
│   │   ├── coordinator.py 
│   │   └── armando_controllers.py
│   └── urdf/
│       └── arm.urdf.xacro      # Manipulator description
```

###🎮 Usage
Quick Start
Follow these steps in separate terminals:

Terminal 1: Launch Gazebo World
```
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
Wait for Gazebo to fully load (~10 seconds)
```
Terminal 2: Launch Navigation Stack
```
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
Wait for "Nav2 is fully active" message 
```
Terminal 3: Spawn Armando Robots
```
source ~/ros2_ws/install/setup.bash
ros2 launch final_project spawn_armando.launch.py
Wait for both robots to spawn and controllers to load (~10-15 seconds)
```
Terminal 4: Start Coordinated Mission
```
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_fra2mo waypoint_follow.launch.py
Mission starts automatically when all robots are ready
```
Monitoring the Mission
In a 5th terminal, monitor robot status:

```
# Watch rover status
ros2 topic echo /rover/status

# Watch Armando 1 status
ros2 topic echo /armando_1/status

# Watch Armando 2 status
ros2 topic echo /armando_2/status

# List all active topics
ros2 topic list
```
###⚙️ Configuration
Navigation Parameters
Edit ros2_fra2mo/config/navigation.yaml to tune:

DWB Controller: Velocity limits, trajectory sampling

Costmaps: Inflation radius, obstacle detection

Planner: Path planning algorithm (NavFn)

Behavior Server: Recovery behaviors

Key parameters:

```
controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.4
    min_vel_x: 0.0
    max_vel_theta: 1.5
Armando Waypoints
Modify waypoints in final_project/scripts/armando_controllers.py:

python
# Armando 1 (Right column)
self.waypoints = {
    'home': [0.0, 0.0, 0.0, 0.0],
    'hammer': [1.57, -1.6, 0.0, 0.0],
    'lift': [0.0, 0.0, 0.0, 0.0],
    'fra2mo': [0.0, -1.6, 0.0, 0.0],
}
Rover Waypoints
Edit waypoints in ros2_fra2mo/scripts/waypoint_navigation_coordinated.py:

python
WAYPOINTS = [
    {'name': 'Waypoint_1', 'x': 1.45, 'y': -0.111, 'yaw': 1.57},
    {'name': 'Waypoint_2', 'x': 1.27, 'y': -2.42, 'yaw': 1.57},
    {'name': 'Waypoint_3', 'x': 1.45, 'y': -0.111, 'yaw': 0.0},
    {'name': 'Home', 'x': 1.46, 'y': -0.91, 'yaw': 3.14},
]
```
###🏗️ Architecture
System Architecture
```
┌─────────────────────────────────────────────────────────┐
│              Mission Coordinator (State Machine)         │
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐     │
│  │ INIT │→│ ARM1 │→│ NAV1 │→│ ARM2 │→│ NAV2 │→ END │
│  └──────┘  └──────┘  └──────┘  └──────┘  └──────┘     │
└──────────────┬─────────────────┬─────────────────┬──────┘
               │                 │                 │
        ┌──────▼──────┐   ┌──────▼──────┐   ┌─────▼──────┐
        │  Armando 1  │   │   Fra2mo    │   │ Armando 2  │
        │  Controller │   │ Navigation  │   │ Controller │
        └──────┬──────┘   └──────┬──────┘   └─────┬──────┘
               │                 │                 │
        ┌──────▼──────┐   ┌──────▼──────┐   ┌─────▼──────┐
        │ Position    │   │   Nav2      │   │ Position   │
        │ Controller  │   │   Stack     │   │ Controller │
        └─────────────┘   └─────────────┘   └────────────┘
Communication Topics
Topic	Type	Publisher	Subscriber	Description
/rover/mission_command	String	Coordinator	Rover Nav	Navigation commands
/rover/status	String	Rover Nav	Coordinator	Navigation status
/armando_1/mission_command	String	Coordinator	Armando 1	Manipulation commands
/armando_1/status	String	Armando 1	Coordinator	Task status
/armando_2/mission_command	String	Coordinator	Armando 2	Manipulation commands
/armando_2/status	String	Armando 2	Coordinator	Task status
State Machine States
python
class MissionState(Enum):
    INIT = 0                    # System initialization
    WAITING_ROVER_READY = 1     # Wait for all robots READY
    ARMANDO1_LOADING = 2        # Command Armando 1 to load
    WAITING_ARMANDO1_DONE = 3   # Wait for loading completion
    ROVER_TO_WP1 = 4            # Navigate to waypoint 1
    ROVER_TO_WP2 = 5            # Navigate to waypoint 2
    WAITING_ROVER_AT_WP2 = 6    # Wait at waypoint 2
    ARMANDO2_UNLOADING = 7      # Command Armando 2 to unload
    WAITING_ARMANDO2_DONE = 8   # Wait for unloading completion
    ROVER_TO_WP3 = 9            # Navigate to waypoint 3
    ROVER_TO_HOME = 10          # Return to home position
    MISSION_COMPLETE = 11       # Mission successful
    MISSION_FAILED = 12         # Mission failed
```
###🔧 Troubleshooting
Common Issues
Issue: Controller failed to configure

```
# Solution: Increase spawn delays in spawn_armando.launch.py
# Change period from 5.0 to 7.0 seconds
Issue: Nav2 not active

# Check Nav2 lifecycle nodes
ros2 lifecycle list /controller_server
ros2 lifecycle get /controller_server

# Should show "active"
Issue: Robot spawns underground

# Check Z coordinate in spawn_entity arguments
# Ensure z_pose > 0.0
Issue: Navigation fails immediately

# Verify initial pose is set
# Check that map is loaded in RViz
# Ensure costmaps are visible
Debug Commands
# List all active nodes
ros2 node list

# Check specific node info
ros2 node info /mission_coordinator

# Monitor TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Visualize robot model
ros2 launch urdf_tutorial display.launch.py model:=<path_to_urdf>

# Check joint states
ros2 topic echo /armando_1/joint_states

# View controller status
ros2 control list_controllers -c /armando_1/controller_manager
```
###📊 Performance
Mission Duration: ~2-3 minutes (full workflow)

Navigation Accuracy: ±0.1m position, ±0.2 rad orientation

Manipulation Success Rate: >95% in simulation

Controller Frequency: 20 Hz (DWB), 100 Hz (joint controllers)



👤 Author
EndSword33

GitHub: @EndSword33


📚 References
ROS2 Documentation

Nav2 Documentation

Gazebo Classic Documentation

ROS2 Control Documentation


