#!/home/user2/anaconda3/envs/ros/bin/python

import rospy
import actionlib
import sys, termios, tty
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# Initial positions
arm_joint1_pos = 0.0
gripper_pos = 0.0  # open

# Limits
ARM_STEP = 0.1
GRIPPER_STEP = 0.01
ARM_MIN = -2.5
ARM_MAX = 2.5
GRIPPER_MIN = 0
GRIPPER_MAX = 0.1

def get_key():
    """Capture a single keypress from the keyboard (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Initial full joint state
arm_positions = [0.0, -0.4, 0.0, -1.5, 0.0, 1.5, 0.0]  # or your default

def send_arm_trajectory(joint1_value):
    client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]

    # Update joint1 only
    arm_positions[0] = joint1_value

    point = JointTrajectoryPoint()
    point.positions = arm_positions
    point.time_from_start = rospy.Duration(0.5)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    client.wait_for_result()

def send_hand_trajectory(pos):
    client = actionlib.SimpleActionClient('/hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
    point = JointTrajectoryPoint()

    # One finger opens with +pos, the other closes with -pos
    point.positions = [pos, -pos]  # mirror effect
    point.time_from_start = rospy.Duration(0.5)
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result()

def main():
    global arm_joint1_pos, gripper_pos
    rospy.init_node('keyboard_controlled_panda')

    print("\nKeyboard control started:")
    print("←/→ : Rotate base joint (panda_joint1)")
    print("↑/↓ : Open/close gripper")
    print("q : Quit\n")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'q':
            print("Exiting...")
            break
        elif key == '\x1b':  # Arrow keys prefix
            key2 = sys.stdin.read(2)
            if key2 == '[D':  # Left arrow
                arm_joint1_pos = max(ARM_MIN, arm_joint1_pos - ARM_STEP)
                print(f"Moving arm base left to {arm_joint1_pos:.2f}")
                send_arm_trajectory(arm_joint1_pos)
            elif key2 == '[C':  # Right arrow
                arm_joint1_pos = min(ARM_MAX, arm_joint1_pos + ARM_STEP)
                print(f"Moving arm base right to {arm_joint1_pos:.2f}")
                send_arm_trajectory(arm_joint1_pos)
            elif key2 == '[A':  # Up arrow
                gripper_pos = min(GRIPPER_MAX, gripper_pos + GRIPPER_STEP)
                print(f"Opening gripper to {gripper_pos:.3f}")
                send_hand_trajectory(gripper_pos)
            elif key2 == '[B':  # Down arrow
                gripper_pos = max(GRIPPER_MIN, gripper_pos - GRIPPER_STEP)
                print(f"Closing gripper to {gripper_pos:.3f}")
                send_hand_trajectory(gripper_pos)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
