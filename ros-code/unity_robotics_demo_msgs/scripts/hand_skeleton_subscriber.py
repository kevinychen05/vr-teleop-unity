#!/usr/bin/env python3

import rospy
import actionlib
from unity_robotics_demo_msgs.msg import HandSkeleton, PosRot
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import math

# Arm joint control state
arm_joint1_pos = 0.0  # base rotation
arm_joint4_pos = -1.5  # up-down movement
arm_joint6_pos = 1.5  # vertical wrist movement
gripper_open = None  # None means uninitialized

# Limits for joints
ARM1_MIN = -2.8973
ARM1_MAX = 2.8973
ARM4_MIN = -3.0718
ARM4_MAX = -0.0698
ARM6_MIN = -0.0175
ARM6_MAX = 3.7525

# Scale factors for mapping wrist data to joint positions
ROT_Y_SCALE = -3.0
POS_Y_SCALE = 3.3356

# pos_y baseline for joint4 mapping
WRIST_Y_MIN = -0.7

# Gripper joint limits
GRIPPER_MIN = 0.05
GRIPPER_MAX = 0.1

# Constants
MIN_DIST = 0.035
MAX_DIST = 0.150

# Wrist rotation input range (from skeleton)
ROT_X_MIN = -0.7
ROT_X_MAX = 0.4

# Dictionary to store latest poses
latest_bone_poses = {}

# Optional: mapping BoneId enum values to human-readable names
BONE_ID_NAMES = {
    0:  "Hand_WristRoot",
    1:  "Hand_ForearmStub",
    2:  "Hand_Thumb0",
    3:  "Hand_Thumb1",
    4:  "Hand_Thumb2",
    5:  "Hand_Thumb3",
    6:  "Hand_Index1",
    7:  "Hand_Index2",
    8:  "Hand_Index3",
    9:  "Hand_Middle1",
    10: "Hand_Middle2",
    11: "Hand_Middle3",
    12: "Hand_Ring1",
    13: "Hand_Ring2",
    14: "Hand_Ring3",
    15: "Hand_Pinky0",
    16: "Hand_Pinky1",
    17: "Hand_Pinky2",
    18: "Hand_Pinky3",
    19: "Hand_MaxSkinnable",
    20: "Hand_ThumbTip",
    21: "Hand_IndexTip",
    22: "Hand_MiddleTip",
    23: "Hand_RingTip",
    24: "Hand_PinkyTip",
    25: "Hand_End"
}

def send_arm_trajectory(joint1_value, joint4_value, joint6_value):
    client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]

    # Use default pose for other joints
    joint_positions = [
        joint1_value, -0.4, 0.0,  # joint2 and joint3 stay constant
        joint4_value,
        0.0, joint6_value, 0.0
    ]

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(0.1)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    # client.wait_for_result()

def send_gripper_trajectory(pos):
    client = actionlib.SimpleActionClient('/hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
    
    point = JointTrajectoryPoint()
    point.positions = [pos, pos]  # both joints use the same value
    point.time_from_start = rospy.Duration(0.1)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()
    
    client.send_goal(goal)
    # client.wait_for_result()

def euclidean_distance(p1, p2):
    dx = p1.pos_x - p2.pos_x
    dy = p1.pos_y - p2.pos_y
    dz = p1.pos_z - p2.pos_z
    return math.sqrt(dx**2 + dy**2 + dz**2)

def hand_skeleton_callback(msg):
    global arm_joint1_pos, arm_joint4_pos, arm_joint6_pos, gripper_open
    for i, bone_id in enumerate(msg.bone_ids):
        pose = msg.bone_poses[i]
        latest_bone_poses[bone_id] = pose

    # Control logic based on Hand_WristRoot (ID 0)
    wrist_pose = latest_bone_poses.get(0)
    if wrist_pose:
        # Joint 1: based on rot_y
        target_joint1 = wrist_pose.rot_y * ROT_Y_SCALE
        target_joint1 = max(min(target_joint1, ARM1_MAX), ARM1_MIN)

        # Joint 4: based on pos_y
        target_joint4 = (wrist_pose.pos_y - WRIST_Y_MIN) * POS_Y_SCALE + ARM4_MIN
        target_joint4 = max(min(target_joint4, ARM4_MAX), ARM4_MIN)

        # Joint 6: based on wrist rot_x (flexion/extension)
        # Linear scaling of rot_x to joint6
        raw_joint6 = (wrist_pose.rot_x - ROT_X_MIN) / (ROT_X_MAX - ROT_X_MIN)
        target_joint6 = raw_joint6 * (ARM6_MIN - ARM6_MAX) + ARM6_MAX
        target_joint6 = max(min(target_joint6, ARM6_MAX), ARM6_MIN)

        # Only send if significant change
        if (abs(target_joint1 - arm_joint1_pos) > 0.05 or
            abs(target_joint4 - arm_joint4_pos) > 0.05 or
            abs(target_joint6 - arm_joint6_pos) > 0.05):
            # rospy.loginfo(f"Moving joint1 to {target_joint1:.2f} (rot_y: {wrist_pose.rot_y:.2f}), "
            #               f"joint4 to {target_joint4:.2f} (pos_y: {wrist_pose.pos_y:.2f}), "
            #               f"joint6 to {target_joint6:.2f} (rot_x: {wrist_pose.rot_x:.2f})")
            arm_joint1_pos = target_joint1
            arm_joint4_pos = target_joint4
            arm_joint6_pos = target_joint6
            send_arm_trajectory(arm_joint1_pos, arm_joint4_pos, arm_joint6_pos)

      # Gripper control with state check
    thumb_pose = latest_bone_poses.get(20)
    index_pose = latest_bone_poses.get(21)

    if thumb_pose and index_pose:
        dist = euclidean_distance(thumb_pose, index_pose)
        should_open = dist > 0.05

        if gripper_open is None or should_open != gripper_open:
            if should_open:
                rospy.loginfo(f"Gripper OPEN: thumb-index distance = {dist:.3f}")
                send_gripper_trajectory(0.4)
            else:
                rospy.loginfo(f"Gripper CLOSE: thumb-index distance = {dist:.3f}")
                send_gripper_trajectory(-0.4)
            gripper_open = should_open

def main():
    rospy.init_node("wrist_based_panda_control")
    rospy.Subscriber("/hand_skeleton", HandSkeleton, hand_skeleton_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rospy
# from unity_robotics_demo_msgs.msg import HandSkeleton, PosRot

# # Dictionary to store latest poses
# latest_bone_poses = {}

# # Optional: mapping BoneId enum values to human-readable names
# BONE_ID_NAMES = {
#     0:  "Hand_WristRoot",
#     1:  "Hand_ForearmStub",
#     2:  "Hand_Thumb0",
#     3:  "Hand_Thumb1",
#     4:  "Hand_Thumb2",
#     5:  "Hand_Thumb3",
#     6:  "Hand_Index1",
#     7:  "Hand_Index2",
#     8:  "Hand_Index3",
#     9:  "Hand_Middle1",
#     10: "Hand_Middle2",
#     11: "Hand_Middle3",
#     12: "Hand_Ring1",
#     13: "Hand_Ring2",
#     14: "Hand_Ring3",
#     15: "Hand_Pinky0",
#     16: "Hand_Pinky1",
#     17: "Hand_Pinky2",
#     18: "Hand_Pinky3",
#     19: "Hand_MaxSkinnable",
#     20: "Hand_ThumbTip",
#     21: "Hand_IndexTip",
#     22: "Hand_MiddleTip",
#     23: "Hand_RingTip",
#     24: "Hand_PinkyTip",
#     25: "Hand_End"
# }

# def hand_skeleton_callback(msg):
#     for i, bone_id in enumerate(msg.bone_ids):
#         pose = msg.bone_poses[i]
#         latest_bone_poses[bone_id] = pose

# def print_bone_poses(event):
#     print("\n--- Latest Hand Bone Poses ---")
#     for bone_id, pose in latest_bone_poses.items():
#         name = BONE_ID_NAMES.get(bone_id, f"Bone_{bone_id}")
#         print(f"{name} (ID: {bone_id})")
#         print(f"  Position: ({pose.pos_x:.2f}, {pose.pos_y:.2f}, {pose.pos_z:.2f})")
#         print(f"  Rotation: ({pose.rot_x:.2f}, {pose.rot_y:.2f}, {pose.rot_z:.2f}, {pose.rot_w:.2f})")

# def main():
#     rospy.init_node("hand_pose_dict_subscriber")
#     rospy.Subscriber("/hand_skeleton", HandSkeleton, hand_skeleton_callback)

#     # Print poses every second
#     rospy.Timer(rospy.Duration(1.0), print_bone_poses)

#     rospy.spin()

# if __name__ == '__main__':
#     main()