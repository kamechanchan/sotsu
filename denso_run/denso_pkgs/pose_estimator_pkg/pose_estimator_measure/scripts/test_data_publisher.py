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
import rospy, rospkg, roslib.packages
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
        rospack = rospkg.RosPack()
        self.header = Header()
        self.header.frame_id = "photoneo_center_optical_frame"
        self.pub_topic_name = pub_topic_name
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.pub = rospy.Publisher(self.pub_topic_name, PointCloud2, queue_size=1)
        self.package_path_ = rospack.get_path("pose_estimator_measure")
        self.num_ = 0
        self.x_data = []
        self.y_data = []
        self.loadHDF5()
        self.receive_ok = rospy.set_param("/publish_data/is_ok", True)

    def loadHDF5(self):
        # hdf5_file = h5py.File(self.package_path_ + "/test_datasets" + "/sim_datasets" + "/photoneo_center_v3_optical_frame_HV8.hdf5", "r")
        # hdf5_file = h5py.File(self.package_path_ + "/test_datasets" + "/real_datasets" + "/test_dataset_HV8_2.hdf5", "r")
        hdf5_file_x = h5py.File(self.package_path_ + "/test_datasets" + "/real_datasets" + "/test_dataset_HV8_15degree_x_axis.hdf5", "r")
        hdf5_file_y = h5py.File(self.package_path_ + "/test_datasets" + "/real_datasets" + "/test_dataset_HV8_15degree_y_axis.hdf5", "r")
        hdf5_file_z = h5py.File(self.package_path_ + "/test_datasets" + "/real_datasets" + "/test_dataset_HV8_15degree_z_axis.hdf5", "r")
        # for n in tqdm(range(10)):
        #     pcl = o3d.PointCloud()
        #     pcl_data = hdf5_file["data_" + str(n+1)]['pcl'][()]
        #     pose_data = hdf5_file["data_" + str(n+1)]['pose'][()]
        #     pcl_data = pcl_data[~np.any(np.isnan(pcl_data), axis=1)]
        #     pcl.points = o3d.Vector3dVector(pcl_data)
        #     self.x_data.append(pcl_data)
        #     self.y_data.append(pose_data)
        # #

        for hdf5_file in [hdf5_file_x, hdf5_file_y, hdf5_file_z]:
            for n in range(5):
                pcl = o3d.PointCloud()
                pcl_data = hdf5_file["data_" + str(n+1)]['pcl'][()]
                pose_data = hdf5_file["data_" + str(n+1)]['pose'][()]
                pcl_data = pcl_data[~np.any(np.isnan(pcl_data), axis=1)]
                pcl.points = o3d.Vector3dVector(pcl_data)
                self.x_data.append(pcl_data)
                self.y_data.append(pose_data)


    def publishEveryPcl(self, np_cloud):
        pcl = o3d.PointCloud()
        pcl.points = o3d.Vector3dVector(np_cloud)
        o3d_data = convertCloudFromOpen3dToRos(pcl, self.header)
        self.pub.publish(o3d_data)


    def init_publish(self, index):
        node.publishEveryTf(node.y_data[index])
        node.publishEveryPcl(node.x_data[index])


    def publishEveryTf(self, pose):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "photoneo_center_optical_frame"
        t.child_frame_id = "grand_truth"
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
        node = DummyNode("cropped_pointcloud")
        index = 0

        node.init_publish(0)
        while not rospy.is_shutdown():
            node.receive_ok = rospy.get_param("/publish_data/is_ok")
            if node.receive_ok:
                node.publishEveryTf(node.y_data[index])
                node.publishEveryPcl(node.x_data[index])
                print(index)
                index += 1
                rospy.set_param("/publish_data/is_ok", False)
    except rospy.ROSInterruptException: pass
