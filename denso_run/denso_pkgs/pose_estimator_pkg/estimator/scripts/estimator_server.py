#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/options'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/scripts/function/__init__.py'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/scripts/function/'))
from options.test_options import TestOptions
from models import create_model
from dnn_test import estimation
import function as f


import numpy as np
from scipy import linalg
import time

# ROS
import rospy, rospkg
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
        rospack = rospkg.RosPack()
        self.opt = TestOptions().parse()
        self.opt.dataset_model = rospy.get_param("~object_name", "HV8")
        self.opt.name = rospy.get_param("~name", "PointNet")
        self.opt.dataset_mode = rospy.get_param("~dataset_mode", "pose_estimation")
        self.opt.batch_size = rospy.get_param("~batch_size", 30)
        self.arch = rospy.get_param("~arch", "PointNet_Pose")
        self.opt.arch = self.arch
        self.opt.resolution = rospy.get_param("~resolution", 1024)
        self.opt.num_threads = rospy.get_param("~num_threads", 8)
        self.opt.gpu_id = rospy.get_param("~gpu_id", "1")
        self.package_path = rospack.get_path("estimator")
        self.opt.checkpoints_dir = self.package_path + "/weights"

        self.model = create_model(self.opt)

        self.output_pos_num = 3
        self.output_ori_num = 9

    def run_service(self):
        service = rospy.Service("pose_estimation", PoseEstimate, self.callback)

    def callback(self, req):
        t0 = time.time()
        res = PoseEstimateResponse()

        if  self.arch == "PointNet_Pose":
            data = req.input_cloud.data
            est_pose, est_time = f.predict_pose(self.model, data, "PointNet")
            res.success = True

        elif self.arch == "3DCNN":
            data = req.input_voxel
            est_pose, est_time = f.predict_pose(self.model, data, "C3D_Voxel")
            res.success = True

        else:
            print("Error while pose prediction operation")
            res.success = False

        res.trans.transform.translation = est_pose.pose.position
        res.trans.transform.rotation = est_pose.pose.orientation
        res.trans.header.stamp = rospy.Time.now()
        res.stamp = est_time

        return res

if __name__ == "__main__":
    node = DnnNode()
    try:
        node.run_service()
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()


