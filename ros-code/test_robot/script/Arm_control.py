#!/home/user2/anaconda3/envs/ros/bin/python

import rospy
import actionlib
import sys, termios, tty
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# Constants
JOINT_LIMITS = [(-2.5, 2.5)] * 7
ARM_STEP = 0.1
GRIPPER_STEP = 0.01
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.1

# Initial joint positions
arm_positions = [0.0, -0.4, 0.0, -1.5, 0.0, 1.5, 0.0]
gripper_pos = 0.0
selected_joint = 0  # default to joint1

def get_key():
    """Capture a single keypress from the keyboard (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # Check for escape sequence (arrow keys)
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def send_arm_trajectory():
    client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]

    point = JointTrajectoryPoint()
    point.positions = arm_positions
    point.time_from_start = rospy.Duration(0.5)

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    client.wait_for_result()

def send_hand_trajectory_custom(finger1, finger2):
    client = actionlib.SimpleActionClient('/hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']
    point = JointTrajectoryPoint()
    point.positions = [finger1, finger2]
    point.time_from_start = rospy.Duration(0.5)
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result()


def main():
    global selected_joint, gripper_pos
    rospy.init_node('keyboard_controlled_panda')

    print("\nKeyboard control started:")
    print("1-7 : Select joint")
    print("a/d : Decrease/Increase selected joint angle")
    print("o   : Open gripper")
    print("i   : Close gripper")

    print("q   : Quit\n")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'q':
            print("Exiting...")
            break
        elif key in '1234567':
            selected_joint = int(key) - 1
            print(f"Selected joint {selected_joint + 1}")
        elif key == 'a':
            min_limit, _ = JOINT_LIMITS[selected_joint]
            arm_positions[selected_joint] = max(min_limit, arm_positions[selected_joint] - ARM_STEP)
            print(f"Joint {selected_joint+1} decreased to {arm_positions[selected_joint]:.2f}")
            send_arm_trajectory()
        elif key == 'd':
            _, max_limit = JOINT_LIMITS[selected_joint]
            arm_positions[selected_joint] = min(max_limit, arm_positions[selected_joint] + ARM_STEP)
            print(f"Joint {selected_joint+1} increased to {arm_positions[selected_joint]:.2f}")
            send_arm_trajectory()
        elif key == 'o':  # Open gripper
            print("Gripper OPENING [0.4, 0]")
            send_hand_trajectory_custom(0.4, 0.4)
        elif key == 'i':  # Close gripper
            print("Gripper CLOSING [0, -0.4]")
            send_hand_trajectory_custom(-0.4, -0.4)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
