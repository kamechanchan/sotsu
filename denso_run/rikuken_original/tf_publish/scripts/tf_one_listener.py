#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs

if __name__=='__main__':
    rospy.init_node("dgvdf")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    late = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('shiny', 'world', rospy.Time())
            kiense = tfBuffer.lookup_transform('naoki', 'world', rospy.Time())
            print(str(trans))
            print(str(kiense))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            late.sleep()
            continue
        