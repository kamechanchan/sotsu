#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

def callback(msgs):
    print(msgs.fields)

rospy.init_node("field")
sub = rospy.Subscriber("all_cloud", PointCloud2, callback)
rospy.spin()