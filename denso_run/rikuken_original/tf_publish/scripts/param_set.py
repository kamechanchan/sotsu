#!/usr/bin/env python3
import rospy
if __name__=='__main__':
    rospy.init_node('node')
    para = rospy.get_param('/os/is_ok', True)
    count = 1
    while count < 10:
        init = input()
        if count % 2 == 0:
            rospy.set_param('/os/is_ok', True)  
        else:
            rospy.set_param('/os/is_ok', False)
        count = count + 1
        