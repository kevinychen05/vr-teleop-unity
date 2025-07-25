#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose

def main():
    rospy.init_node('cube_pose_publisher')
    pub = rospy.Publisher('/cube_pose', Pose, queue_size=10)

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            response = get_model_state('small_cube', '')  # Replace 'cube' with your model name
            pose = response.pose
            pub.publish(pose)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        rate.sleep()

if __name__ == '__main__':
    main()
