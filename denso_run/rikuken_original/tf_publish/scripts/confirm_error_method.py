#!/usr/bin/env python3
from pyquaternion import Quaternion
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation 
from tf.transformations import quaternion_matrix, euler_matrix
import tf
import tf2_ros
import tf2_msgs
import rospy
import time

if __name__=='__main__':
    rospy.init_node('erroe_node')
    limit = rospy.get_param('~limit_time', 10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)
    count = 1
    start_time = time.time()
    theta_sum = 0
    ros_euler = [0, 0, 0]
    quat_euler = [0, 0, 0]
    axis_sum = [0, 0, 0]
    result_euler = [0, 0, 0]
    kakeru = 180 / pi
    while 1:
        process_time = time.time()
        if process_time - start_time > limit:
            break
        try:
            error_result = tfBuffer.lookup_transform('HV8', 'estimated_tf', rospy.Time(0))
            error_trans = error_result.transform.translation
            error_rot = error_result.transform.rotation
            my_quaternion = Quaternion(error_rot.w, error_rot.x, error_rot.y, error_rot.z)
            u = my_quaternion.axis
            #print(str(u))
            theta = my_quaternion.degrees
            print(str(theta))
            theta_sum = theta_sum + theta
            
           # print(str(my_quaternion.yaw_pitch_roll[2]*kakeru) + ' ' + str(my_quaternion.yaw_pitch_roll[1]*kakeru) + ' ' + str(my_quaternion.yaw_pitch_roll[0]*kakeru))
            euler = tf.transformations.euler_from_quaternion((error_rot.x, error_rot.y, error_rot.z, error_rot.w))
            #print(str(euler[0]*kakeru) + ' ' + str(euler[1]*kakeru) + ' ' + str(euler[2]*kakeru))
            for i in range(3):
                ros_euler[i] = ros_euler[i] + euler[i]
                quat_euler[i] = quat_euler[i] + my_quaternion.yaw_pitch_roll[i]
                axis_sum[i] = axis_sum[i] + u[i]
                result_euler[i] = result_euler[i] + (my_quaternion.yaw_pitch_roll[2-i] *kakeru)
            count = count + 1
            rate.sleep()    
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    print('average is ' + str(theta_sum / count))
    error_roll = (ros_euler[0] - quat_euler[2]) * kakeru / count
    error_pitch = (ros_euler[1] - quat_euler[1]) * kakeru / count
    error_yaw = (ros_euler[2] - quat_euler[0]) * kakeru / count
    #print('error between ros and pyquaternion is ' + str(error_roll) + '  ' + str(error_pitch) + '  ' + str(error_yaw))
    #print(str(quat_euler[2] * kakeru / count) + '  ' + str(quat_euler[1] * kakeru / count) + '  ' + str(quat_euler[2] * kakeru / count))
    print(str(axis_sum[0] / count) + '  ' + str(axis_sum[1] / count) + '  ' + str(axis_sum[2] / count))
    print(str(result_euler[0] / count) + '  ' + str(result_euler[1] / count) +  '   ' + str(result_euler[2] / count))