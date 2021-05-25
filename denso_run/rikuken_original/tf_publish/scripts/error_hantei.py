#!/usr/bin/env python3
from operator import pos
from pyquaternion import Quaternion
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, euler_matrix, euler_from_quaternion, quaternion_from_euler
import tf 
import tf2_ros
import tf2_msgs
import rospy
import time
from geometry_msgs.msg import Transform
from gazebo_msgs.msg import *
import rospkg
import random

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
    count = 0
    model_state_pub_ = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pos_ = ModelState()
    pos_.model_name = "HV8"
    file = open(rospack.get_path("estimator") + '/zentai.txt', 'w')
    while not rospy.is_shutdown():
        
        try:
            error_trans = Transform().translation
            #print("receive")
            error_rot = Transform().rotation
            error_result = tfBuffer.lookup_transform('HV8', 'estimated_tf', rospy.Time())
            groud_result = tfBuffer.lookup_transform('world', 'HV8', rospy.Time())
            esti_result = tfBuffer.lookup_transform('world', 'estimated_tf', rospy.Time())
            error_trans = error_result.transform.translation
            error_rot = error_result.transform.rotation
            error_euler = euler_from_quaternion((error_rot.x, error_rot.y, error_rot.z, error_rot.w))
            groud_trans = groud_result.transform.translation
            groud_rot = groud_result.transform.rotation
            groud_euler = euler_from_quaternion((groud_rot.x, groud_rot.y, groud_rot.z, groud_rot.w))
            esti_trans = esti_result.transform.translation
            esti_rot = esti_result.transform.rotation
            esti_euler = euler_from_quaternion((esti_rot.x, esti_rot.y, esti_rot.z, esti_rot.w))

            my_quaternion = Quaternion(error_rot.w, error_rot.x, error_rot.y, error_rot.z)
            u  = my_quaternion.degrees
            count = count  + 1
            print("hi" + str(count))
            if max_hantei(esti_trans.x - groud_trans.x, esti_trans.y - groud_trans.y, esti_trans.z - groud_trans.z, 0.007):
                
                time_start = rospy.get_param("time_start", time.time())
                
                kake = 180 / pi

                file.write("姿勢を移動させていい感じの位置を認識するまでの時間は　: " + str(time.time() - time_start) + '\n')
                file.write("真値　 x: " + str(groud_trans.x) + "  y: " + str(groud_trans.y) + "  z: " + str(groud_trans.z))
                file.write("  roll: " + str(groud_euler[0]*kake) + "   pitch: " + str(groud_euler[1]*kake) + "    yaw: " + str(groud_euler[2]*kake) + " \n")
                file.write("推定値 x: " + str(esti_trans.x) + "  y: " + str(esti_rot.y) + "  z: " + str(esti_rot.z))
                file.write(" roll: " + str(esti_euler[0]*kake) + "  pitch: " + str(esti_euler[1]*kake) + "  yaw: " + str(esti_euler[2]*kake) + " \n")
                file.write("error  x: " + str(error_trans.x) + "  y: " + str(error_trans.y) + "  z: " + str(error_trans.z))
                file.write("  roll: " + str(error_euler[0] * kake) + "  pitch: " + str(error_euler[1] * kake) + "  yaw: " + str(error_euler[2]* kake) + "\n\n\n")
                pos_.pose.position.x = random.uniform(-0.2, 0.2)
                pos_.pose.position.y = random.uniform(-0.2, 0.2)
                pos_.pose.position.z = random.uniform(0.1, 0.25)
                hani = 2
                roll = random.uniform(-pi/hani, pi/hani)
                pitch = random.uniform(-pi/hani, pi/hani)
                yaw = random.uniform(-pi/hani, pi/hani)
                quat = quaternion_from_euler(roll, pitch, yaw)
                pos_.pose.orientation.x = quat[0]
                pos_.pose.orientation.y = quat[1]
                pos_.pose.orientation.z = quat[2]
                pos_.pose.orientation.w = quat[3]
                model_state_pub_.publish(pos_)

                rospy.set_param("time_start", time.time())
            
                
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
            continue
        
        rate.sleep()
    file.close()
