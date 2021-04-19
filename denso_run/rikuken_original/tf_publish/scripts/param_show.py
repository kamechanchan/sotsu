#!/usr/bin/env python3
import rospy

if __name__=='__main__':
    rospy.init_node('name')
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        ten = rospy.get_param('os/is_ok', True)
        rospy.loginfo(ten)
        loop.sleep()