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
import tf


class GT_Listener(object):
    def __init__(self, sub_topic_name):

        rospy.wait_for_service("pose_estimation")

        self.sub_topic_name = sub_topic_name
        self.sensor_frame_id = "photoneo_center_optical_frame"
        self.tfBuffer = tf2_ros.Buffer()
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub = rospy.Subscriber(self.sub_topic_name, PointCloud2, self.callback)
        self.input_data = None

    def callback(self, data):
        header = data.header
        o3d_data = convertCloudFromRosToOpen3d(data)
        point_xyz = random.sample(o3d_data.points, 1024)
        self.input_data = Float32MultiArray(data=np.array(point_xyz).flatten())
        res = self.getPoseData()

        #t = np.array([[res.pose.position.x], [res.pose.position.y], [res.pose.position.z]])
        #quat = np.array([res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w])


#        print("Pose Estimated\n")
#        print('{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}, {4:.2f}, {5:2f}'.format(
#            res.pose.position.x,
#            res.pose.position.y,
#            res.pose.position.z,
#            res.pose.orientation.x,
#            res.pose.orientation.y,
#            res.pose.orientation.z,
#        ))
#
        try:
            gt = self.tfBuffer.lookup_transform(self.sensor_frame_id, "grand_truth", rospy.Time())
#            print("Grand Truth\n")
#            print('{0:.2f}, {1:.2f}, {2:.2f}, {3:.2f}, {4:.2f}, {5:2f}'.format(
#                gt.transform.translation.x,
#                gt.transform.translation.y,
#                gt.transform.translation.z,
#                gt.transform.rotation.x,
#                gt.transform.rotation.y,
#                gt.transform.rotation.z,
#            ))
#
            print("Error")
            quaternion_es = (
            res.pose.orientation.x,
            res.pose.orientation.y,
            res.pose.orientation.z,
            res.pose.orientation.w)

            quaternion_gt = (
            gt.transform.rotation.x,
            gt.transform.rotation.y,
            gt.transform.rotation.z,
            gt.transform.rotation.w)

            euler_res = tf.transformations.euler_from_quaternion(quaternion_es)
            euler_gt = tf.transformations.euler_from_quaternion(quaternion_gt)

            roll = euler_res[0] - euler_gt[0]
            pich = euler_res[1] - euler_gt[1]
            yaw = euler_res[2] - euler_gt[2]

            print('{0:.5f}, {1:.5f}, {2:.5f}, {3:5f}, {4:5f}, {5:6f}'.format(
                res.pose.position.x - gt.transform.translation.x,
                res.pose.position.y - gt.transform.translation.y,
                res.pose.position.z - gt.transform.translation.z,
                roll,
                pich,
                yaw
            ))



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass

    def Publisher(self, res):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.sensor_frame_id
        t.child_frame_id = "estimated_tf"
        t.transform.translation = res.pose.position
        t.transform.rotation = res.pose.orientation
        self.tf.sendTransform(t)

    def getPoseData(self):
        try:
            client = rospy.ServiceProxy("pose_estimation", PoseEstimate)
            req = PoseEstimateRequest()
            res = PoseEstimateResponse()
            req.input_cloud = self.input_data
            res = client(req)
            self.Publisher(res)
            return res
        except rospy.ServiceException, e:
            rospy.loginfo("Fail.")


if __name__ == '__main__':

    rospy.init_node('cropped_cloud_repulisher')
    node = GT_Listener("cropped_pointcloud")

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
