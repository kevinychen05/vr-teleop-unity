# VR-Based Teleoperation of a Simulated Robotic Arm Using Meta Quest 3 Hand Tracking

This project enables remote robotic manipulation using hand tracking from the Meta Quest 3 in Unity, streamed via ROS to a robotic arm simulated in Gazebo. It is designed for research in human-robot interaction, VR-robotics, and teleoperation — such as use cases like bomb squad training or remote manipulation tasks.

---

## Requirements

This project was developed and tested on Windows 11 and Ubuntu 20.04 with:

- **ROS Noetic**
- **Unity 6.1.3**
- **Meta Quest 3**
- **Meta XR SDK + Interaction SDK**
- **ROS–Unity integration** (via ROS-TCP-Connector and Unity Robotics Hub)

---

## Getting Started

Follow these steps to get everything running:

### 1. Clone the Repository

```bash
git clone https://github.com/kevinychen05/vr-teleop-unity.git
cd vr-teleop-unity
```

### 2. Set Up the ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/path/to/vr-teleop-unity/ros-code/unity_robotics_demo_msgs .
ln -s ~/path/to/vr-teleop-unity/ros-code/test_robot .
cd ..
catkin_make
source devel/setup.bash
```

Replace `~/path/to/vr-teleop-unity` with the actual path to this repo on your machine.
