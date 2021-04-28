#!/usr/bin/env python3
import rospy
import tf2_ros
import tf
import tf2_msgs
import numpy as np
import math
import time

def space_(space_num):
    for i in range(space_num):
        print('')

def print_result(result_array, result_string):
    str_res = result_string + ' is : '
    for i in range(3):
        str_res = str_res + '  {:.5f}'.format(result_array[i])
    for i in range(3):
        str_res = str_res + ' {:.2f}'.format(result_array[i + 3] * 180 / math.pi)
    print(str_res)

def print_3(n1, n2, n3):
    print(str(n1) + '   ' + str(n2) + '    ' + str(n3))

if __name__=='__main__':
    rospy.init_node('error_node')
    limit = rospy.get_param("~limit_time", 10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10)
    count = 1
    euler_error = []
    euler_error_max = []
    euler_error_min = []
    euler_error_average = []
    euler_error_sum = []
    start_time = time.time()
    while 1:
        process_time = time.time()
        if process_time - start_time > limit:
            break
        try:
            euler_error.clear()
            estimation = tfBuffer.lookup_transform('world', 'estimated_tf', rospy.Time(0))
            groud_truth = tfBuffer.lookup_transform('world', 'HV8', rospy.Time(0))
            esti_trans = estimation.transform.translation
            groud_truth_trans = groud_truth.transform.translation
            esti_rot = estimation.transform.rotation
            groud_rot = groud_truth.transform.rotation
            esti_matrix = tf.transformations.quaternion_matrix(np.array([esti_rot.x, esti_rot.y, esti_rot.z, esti_rot.w]))
            groud_matrix = tf.transformations.quaternion_matrix(np.array([groud_rot.x, groud_rot.y, groud_rot.z, groud_rot.w]))
            esti_matrix = np.asarray(esti_matrix)
            groud_matrix = np.asarray(groud_matrix)
            error_matrix = np.dot(esti_matrix, groud_matrix.T)
            #print('error matrix is')
            #print(error_matrix[0:3, 0:3])
            #print('groud_Truth is x %f  y %f z %f' % (groud_truth_trans.x, groud_truth_trans.y, groud_truth_trans.z))
            #print('estimat     is x %f  y %f z %f' % (esti_trans.x, esti_trans.y, esti_trans.z))
            euler_error.append(groud_truth_trans.x - esti_trans.x)
            euler_error.append(groud_truth_trans.y - esti_trans.y)
            euler_error.append(groud_truth_trans.z - esti_trans.z)
            euler = tf.transformations.euler_from_matrix(error_matrix)
            for i in range(3):
                euler_error.append(euler[i])
            if count == 1:
                for i in range(6):
                    euler_error_min.append(abs(euler_error[i]))
                    euler_error_max.append(abs(euler_error[i]))
                    euler_error_sum.append(abs(euler_error[i]))
            else:
                for i in range(6):
                    if abs(euler_error[i]) > abs(euler_error_max[i]):
                        euler_error_max[i] = abs(euler_error[i])
                    if abs(euler_error[i]) < abs(euler_error_min[i]):
                        euler_error_min[i] = abs(euler_error[i])
                    euler_error_sum[i] = euler_error_sum[i] + abs(euler_error[i])
            euler_error_average.clear()
            #print(count)
            count = count + 1
            rate.sleep()    
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
    for i in range(6):
        euler_error_average.append(euler_error_sum[i] / (count - 1))
    print_result(euler_error_max, 'max_error')
    print_result(euler_error_min, 'min_error')
    print_result(euler_error_average, 'ave_error')
