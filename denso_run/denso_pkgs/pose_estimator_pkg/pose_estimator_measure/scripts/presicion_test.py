#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from cloud_util import *
import random
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateRequest, PoseEstimateResponse
from pose_estimator_msgs.msg import InputData
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
import tf, rospkg
import pandas as pd
import math
from tf_sync import TfMessageFilter
import message_filters


class log_data(object):
    def __init__(self, file_name):
        self.file_name = file_name
        self.log = pd.DataFrame([], columns=["x", "y", "z", "roll", "pitch", "yaw"], index=range(20))

    def set_log(self, num_array , index):
        self.log.iloc[index, 0] = num_array[0]
        self.log.iloc[index, 1] = num_array[1]
        self.log.iloc[index, 2] = num_array[2]
        self.log.iloc[index, 3] = num_array[3]
        self.log.iloc[index, 4] = num_array[4]
        self.log.iloc[index, 5] = num_array[5]

    def save_log(self):
        self.log.to_csv(self.file_name)


class Precision_test(object):
    def __init__(self, sub_topic_name):
        rospack = rospkg.RosPack()
        self.sub_topic_name = sub_topic_name
        self.sensor_frame_id = "photoneo_center_optical_frame"
        self.tfBuffer = tf2_ros.Buffer()
        self.package_path = rospack.get_path("pose_estimator_measure")
        self.tf = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.sub_topic_name, PointCloud2, self.callback)
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub_pcd = message_filters.Subscriber(self.sub_topic_name, PointCloud2)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.sub_pcd], 10, 0.01)
        self.ts = TfMessageFilter(self.sync_sub, "grand_truth", "icpRefine", queue_size=100)
        self.input_data = None
        self.df_est_log = log_data(self.package_path + "/result/est_error_predision.csv")
        self.df_refine_log = log_data(self.package_path + "/result/refine_error_predision.csv")
        self.index = 0
        self.flag = 0

    def callback(self, data):
        self.receive_ok = rospy.get_param("/publish_data/is_ok")
        if self.receive_ok:
            rospy.set_param("/publish_data/is_ok", True)
            print("test")
            return False

        try:
            if self.flag == 0:
                self.flag = 1
                return 

            time.sleep(1)
            est_tf_err = self.tfBuffer.lookup_transform("grand_truth", "estimated_tf", rospy.Time())
            refine_tf_err = self.tfBuffer.lookup_transform("grand_truth", "icpRefine", rospy.Time())

            gt2est_trans_error = (
            est_tf_err.transform.translation.x,
            est_tf_err.transform.translation.y,
            est_tf_err.transform.translation.z)

            gt2est_rot_error = (
            est_tf_err.transform.rotation.x,
            est_tf_err.transform.rotation.y,
            est_tf_err.transform.rotation.z,
            est_tf_err.transform.rotation.w)

            gt2refine_trans_error = (
            refine_tf_err.transform.translation.x,
            refine_tf_err.transform.translation.y,
            refine_tf_err.transform.translation.z)


            gt2refine_rot_error = (
            refine_tf_err.transform.rotation.x,
            refine_tf_err.transform.rotation.y,
            refine_tf_err.transform.rotation.z,
            refine_tf_err.transform.rotation.w)

            euler_est = tf.transformations.euler_from_quaternion(gt2est_rot_error)
            euler_refine = tf.transformations.euler_from_quaternion(gt2refine_rot_error)

            error_est_log = np.array([1000 * gt2est_trans_error[0], 1000 * gt2est_trans_error[1], 1000 * gt2est_trans_error[2], math.degrees(euler_est[0]), math.degrees(euler_est[1]), math.degrees(euler_est[2])], dtype="float32")

            error_refine_log = np.array([1000 * gt2refine_trans_error[0], 1000 * gt2refine_trans_error[1], 1000 * gt2refine_trans_error[2], math.degrees(euler_refine[0]), math.degrees(euler_refine[1]), math.degrees(euler_refine[2])], dtype="float32")


            print(error_est_log)
            print(error_refine_log)
            self.df_est_log.set_log(error_est_log, self.index)
            self.df_refine_log.set_log(error_refine_log, self.index)
            self.df_est_log.save_log()
            self.df_refine_log.save_log()
            self.index += 1
            print("save to precision")
            print(self.index)
            rospy.set_param("/publish_data/is_ok", True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass


if __name__ == '__main__':

    rospy.init_node('precision_test_node')
    node = Precision_test("refine_cloud")
    rospy.spin()
