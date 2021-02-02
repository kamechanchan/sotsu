#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))

from options.test_options import TestOptions
from dnn_test import estimation
import function as f

import numpy as np
from scipy import linalg
import time

# ROS
import rospy
import roslib.packages
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateResponse
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix


class DnnNode():
    def __init__(self):
        rospy.init_node("Pose_Estimation_server")
        rospy.loginfo("Ready.")

        self.opt = TestOptions().parse()
        self.opt.name = "PointNet"
        self.opt.dataset_mode = "pose_estimation"
        self.opt.batch_size = 30
        self.opt.arch = "PointNet"
        self.opt.num_threads = 8
        self.opt.gpu_id = "0"
        self.opt.checkpoints_dir = "../../weights"

        self.output_pos_num = 3
        self.output_ori_num = 9
        service = rospy.Service("pose_estimation", PoseEstimate, self.callback)

    def callback(self, req):
        t0 = time.time()
        res = PoseEstimateResponse()

        if  req.input_cloud.data:
            data = req.input_cloud.data
            y_pre = f.predict_pose(self.opt, data, "PointNet")
            y = y_pre[0].squeeze()
            op_time = y_pre[1]
            res.success = True
            res.pose.position.x = y[0]
            res.pose.position.y =  y[1]
            res.pose.position.z =  y[2]
            res.pose.orientation.x = y[3]
            res.pose.orientation.y = y[4]
            res.pose.orientation.z = y[5]
            res.pose.orientation.w = y[6]
            res.stamp = op_time

            return res

        elif req.input_voxel.data:
            data = req.input_voxel
            y_pre = f.predict_pose(self.opt, data, "C3D_Voxel")


if __name__ == "__main__":

    try:
        node = DnnNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass


