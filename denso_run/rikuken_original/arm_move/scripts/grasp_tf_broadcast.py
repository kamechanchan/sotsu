#/usr/bin/env python3
import rospy
import math
from rospy.timer import Rate
import tf2_ros
from geometry_msgs.msg import *

def tf_get(tfBuffer, source, target):
    rage = rospy.Rate(10)
    while 1:
        try:
            trans = tfBuffer.lookup_transform(target, source, rospy.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rage.sleep()
            continue
    return trans

if __name__=='__main__':
    rospy.init_node("tsuchida")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    object_frame = rospy.get_param("object_frame", "HV8_0")
    object_trans = tf_get(tfBuffer, "world", object_frame)
    broad = geometry_msgs.msg.TransformStamped()
    broad.header.frame_id = "world"
    broad.child_frame_id = "tsuchida"
    
