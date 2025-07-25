#!/home/user2/anaconda3/envs/ros/bin/python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def send_arm_trajectory():
    arm_client = actionlib.SimpleActionClient('/panda_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for arm controller...")
    arm_client.wait_for_server()

    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]

    point = JointTrajectoryPoint()
    point.positions = [0.0, -0.4, 0.0, -1.5, 0.0, 1.5, 0.0]
    point.time_from_start = rospy.Duration(3.0)
    arm_goal.trajectory.points.append(point)

    arm_goal.trajectory.header.stamp = rospy.Time.now()
    rospy.loginfo("Sending arm trajectory...")
    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result()
    rospy.loginfo("Arm movement complete.")

def send_hand_trajectory():
    import actionlib
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from trajectory_msgs.msg import JointTrajectoryPoint

    hand_client = actionlib.SimpleActionClient('/hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for hand controller...")
    hand_client.wait_for_server()

    hand_goal = FollowJointTrajectoryGoal()
    hand_goal.trajectory.joint_names = ['panda_finger_joint1', 'panda_finger_joint2']

    point = JointTrajectoryPoint()
    point.positions = [0.03, 0.03]  # open gripper symmetrically
    point.time_from_start = rospy.Duration(2.0)
    hand_goal.trajectory.points.append(point)

    hand_goal.trajectory.header.stamp = rospy.Time.now()
    rospy.loginfo("Sending hand trajectory to both fingers...")
    hand_client.send_goal(hand_goal)
    hand_client.wait_for_result()
    rospy.loginfo("Hand movement complete.")


if __name__ == '__main__':
    rospy.init_node('panda_arm_and_hand_controller')
    send_arm_trajectory()
    send_hand_trajectory()
