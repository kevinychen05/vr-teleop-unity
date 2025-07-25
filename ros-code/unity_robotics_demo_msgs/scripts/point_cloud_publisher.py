#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import random

def create_fake_point_cloud():
    # Create a list of fake points (x, y, z)
    points = []
    for _ in range(100):  # You can increase this number
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)
        z = random.uniform(0.5, 1.0)
        points.append([x, y, z])

    # Create header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"  # You can change this to match your Unity reference frame

    # Define fields for x, y, z (float32)
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    # Create PointCloud2 message
    cloud = pc2.create_cloud(header, fields, points)
    return cloud

def point_cloud_publisher():
    rospy.init_node('fake_point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('/depth/points', PointCloud2, queue_size=10)
    rate = rospy.Rate(5)  # 5 Hz

    while not rospy.is_shutdown():
        cloud_msg = create_fake_point_cloud()
        pub.publish(cloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        point_cloud_publisher()
    except rospy.ROSInterruptException:
        pass
