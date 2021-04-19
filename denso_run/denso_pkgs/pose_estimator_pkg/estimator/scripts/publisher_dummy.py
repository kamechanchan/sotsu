#!/usr/bin/python
# -*- coding: utf-8 -*-
import os, sys, h5py, time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../gen_dataset'))
from scripts.util import *
import open3d as o3d

from tqdm import *
import numpy as np
from scipy import linalg

# ROS
import rospy, roslib.packages
import tf2_ros
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateRequest
from pose_estimator_msgs.msg import InputData
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix


class DummyNode():
    def __init__(self, pub_topic_name):
        rospy.init_node("Dummy_node", anonymous=True)
        self.header = Header()
        self.header.frame_id = "photoneo_center_v1_optical_frame"
        self.pub_topic_name = pub_topic_name
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.pub = rospy.Publisher(self.pub_topic_name, PointCloud2, queue_size=1)
        self.num_ = 0
        self.x_data = []
        self.y_data = []
        self.loadHDF5()

    def loadHDF5(self):
        hdf5_file = h5py.File("../../datasets/photoneo_center_v1_optical_frame_HV6.hdf5", "r")
        for n in tqdm(range(1, 100)):
            pcl = o3d.PointCloud()
            pcl_data = hdf5_file["data_" + str(n)]['pcl'][()]
            pose_data = hdf5_file["data_" + str(n)]['pose'][()]
            pcl_data = pcl_data[~np.any(np.isnan(pcl_data), axis=1)]
            pcl.points = o3d.Vector3dVector(pcl_data)
            self.x_data.append(pcl_data)
            self.y_data.append(pose_data)


    def publishEveryPcl(self, np_cloud):
        pcl = o3d.PointCloud()
        pcl.points = o3d.Vector3dVector(np_cloud)
        o3d_data = convertCloudFromOpen3dToRos(pcl, self.header)
        self.pub.publish(o3d_data)


    def publishEveryTf(self, pose):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "photoneo_center_v1_optical_frame"
        t.child_frame_id = "dummy_tf"
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]
        self.tf.sendTransform(t)
        

if __name__ == "__main__":
    try:
        node = DummyNode("dummy_cloud")
        index = 0
        while not rospy.is_shutdown():
            pass
            node.publishEveryTf(node.y_data[index])
            node.publishEveryPcl(node.x_data[index])
            index += 1
            print(index)
            time.sleep(0.5)

    except rospy.ROSInterruptException: pass


