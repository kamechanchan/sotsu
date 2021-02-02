#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../gen_dataset'))

from options.test_options import TestOptions
from test import *
from scripts.util import *
import open3d as o3d

import numpy as np
from scipy import linalg
import time
import random

# ROS
import rospy, roslib.packages
import tf2_ros
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateRequest
from pose_estimator_msgs.msg import InputData
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix


class PoseEstNode():
    def __init__(self, sub_topic_name):
        rospy.init_node("Pose_Estimation_client", anonymous=True)
        rospy.wait_for_service("pose_estimation")

        self.sub_topic_name = sub_topic_name
        self.opt = TestOptions().parse()
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.sub = rospy.Subscriber(self.sub_topic_name, PointCloud2, self.callback)
        self.input_data = None

    def callback(self, data):
        o3d_data = convertCloudFromRosToOpen3d(data)
        point_xyz = random.sample(o3d_data.points, 1000)
        self.input_data = Float32MultiArray(data=np.array(point_xyz).flatten())

        #voxel_data = data.input_voxel
        #self.input_data = Float32MultiArray(data=np.asarray(voxel).flatten())

        res = self.getPoseData()

    def getPointData(self):
        pcd = o3d.io.read_point_cloud("./hv8_1.ply")
        xyz = random.sample(pcd.points, 1000)
        self.input_data = Float32MultiArray(data=np.asarray(xyz).flatten())

    def getVoxelData(self):
        self.input_data = Float32MultiArray(data=np.asarray(voxel).flatten())

    def Publisher(self, res):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "photoneo_center_optical_frame"
        t.child_frame_id = "estimated_tf"
        t.transform.translation = res.pose.position
        t.transform.rotation = res.pose.orientation
        self.tf.sendTransform(t)

    def getPoseData(self):
        try:
            client = rospy.ServiceProxy("pose_estimation", PoseEstimate)
            req = PoseEstimateRequest()
            req.input_cloud = self.input_data
            res = client(req)
            print(res)
            self.Publisher(res)
            rospy.loginfo("Success.time : %f", res.stamp)
            return res
        except rospy.ServiceException, e:
            rospy.loginfo("Fail.")


if __name__ == "__main__":
    try:
        node = PoseEstNode("cropped_pointcloud")
        rospy.spin()
    except rospy.ROSInterruptException: pass


