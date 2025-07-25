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

### 3. Launch the Robot Simulation

```bash
roslaunch test_robot gazebo.launch
```

The configuration of the Gazebo environment, including all the models and objects in the scene, can be found in `ros-code/test_robot/launch/Hp_test_uniwa_campus.world`.

### 4. Run ROS Nodes

After launching the robot simulation, open new terminals (sourced with `setup.bash`) and run the following:

Hand Tracking Subscriber (controls robot with your hand):

```bash
rosrun unity_robotics_demo_msgs hand_skeleton_subscriber.py
```

Cube Pose Publisher (syncs Gazebo cube to Unity cube):

```bash
rosrun test_robot cube_pose_publisher.py
```

These two nodes ensure that:
- Your Meta Quest 3 hand motion controls the robot arm in Gazebo
- The cube in Gazebo is tracked and mirrored by the cube in Unity

### 5. Run the Unity Scene

- Open `vr-teleop-unity/unity-project` in Unity
- If not already done, configure your Unity project for Meta XR development by following the steps outlined [here](https://developers.meta.com/horizon/documentation/unity/unity-tutorial-hello-vr) up until (not including) the section titled "Add the Meta XR camera rig".
- If not already done, set up ROS-Unity integration by following the steps outlined [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md)
- Connect your Meta Quest 3
- Build and run the scene in Unity
- Move your right hand — the simulated robot will mirror your hand motions in Gazebo. If you make a grabbing gesture (like a fist), the robot should mirror that by closing its claws. Now try picking up the cube and moving it around — the cube you see through the Meta Quest should follow suit.

  > **Note:** There are several code files in this repo that are irrelevant (used for earlier testing) or that came with demo packages such as Unity Robotics Hub's ROS-Unity integration tutorials. The most important files are `HandSkeletonPublisher.cs` and `CubePoseSubscriber.cs` under `unity-project/Assets/Scripts`, as well as `hand_skeleton_subscriber.py` under `ros-code/unity_robotics_demo_msgs/scripts` and `cube_pose_publisher.py` under `ros-code/test_robot/script`.

