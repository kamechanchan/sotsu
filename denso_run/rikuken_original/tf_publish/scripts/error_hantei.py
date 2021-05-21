#!/usr/bin/env python3
from pyquaternion import Quaternion
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, euler_matrix
import tf 
import tf2_ros
import tf2_msgs
import rospy
import time
from geometry_msgs.msg import Transform
import rospkg

def max_hantei(p1, p2, p3, n):
    if p1 < n and p2 < n and p3 < n:
        return True
    else:
        return False



if __name__=='__main__':
    rospy.init_node('error_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30)
    rospack = rospkg.RosPack()
    file = open(rospack.get_path("estimator") + '/zentai.txt', 'w')
    while not rospy.is_shutdown():
        receive = rospy.get_param("/HV8/receive_cloud/is_ok", True)
        if receive:
            try:
                error_trans = Transform().translation
                error_rot = Transform().rotation
                error_result = tfBuffer.lookup_transform('HV8', 'estimated_tf', rospy.Time())

                error_trans = error_result.transform.translation
                error_rot = error_result.transform.rotation
                my_quaternion = Quaternion(error_rot.w, error_rot.x, error_rot.y, error_rot.z)
                u  = my_quaternion.degrees
                hne = rospy.get_param("/HV8/receive_cloud/is_ok")
                if max_hantei(error_trans.x, error_trans.y, error_trans.z, 0.007) and u < 40:
                    rospy.set_param("/HV8/record_cloud/is_ok", True)
                    time_start = rospy.get_param("time_start")
                    if hne:
                        file.write("姿勢を移動させていい感じの位置を認識するまでの時間は　: " + str(time.time() - time_start) + '\n\n\n\n')
                        rospy.set_param("time_start", time.time())
                else:
                    rospy.set_param("/HV8/record_cloud/is_ok", False)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                
                continue
            rospy.set_param("/HV8/receive_cloud/is_ok", False)
        rate.sleep()
