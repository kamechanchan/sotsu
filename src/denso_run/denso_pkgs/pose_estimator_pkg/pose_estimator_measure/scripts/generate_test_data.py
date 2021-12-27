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
import time, h5py
import numpy as np
from scipy.spatial.transform import Rotation
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateRequest, PoseEstimateResponse
from pose_estimator_msgs.msg import InputData
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
import tf, rospkg, ros_numpy
from tf_sync import TfMessageFilter
import message_filters



class GenTestData(object):
    def __init__(self, sub_topic_name):
        rospack = rospkg.RosPack()
        self.sub_topic_name = sub_topic_name
        self.sensor_frame_id = "photoneo_center_optical_frame"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub_pcd = message_filters.Subscriber(self.sub_topic_name, PointCloud2)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.sub_pcd], 10, 0.01)
        self.ts = TfMessageFilter(self.sync_sub, self.sensor_frame_id, "grand_truth", queue_size=100)
        self.path = rospack.get_path("pose_estimator_measure")
        self.save_path = self.path + "/" + "test_datasets" + "/" + "real_datasets" + "/" + "test_dataset_" + sys.argv[1]
        self.save_num = 1
        self.test_data_num = 5
        self.init_hdf5(self.save_path)

    def init_hdf5(self, file_path):
        file_path = file_path + ".hdf5"
        self.hdf5_file = h5py.File(file_path, 'w')

    def callback(self, point_cloud, tf_data):
        try:
            translation = np.array(tf_data[0])
            rotation = np.array(tf_data[1])
            pose = np.concatenate([translation, rotation])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Error")
            pass

        print("Do you want to save this data?")
        print("You shold skip saving data initially")
        inp=True if raw_input('y/n? >> ')=='y' else False
        if (inp):
            self.savePCDandPose(point_cloud, pose)
        else:
            print("Skip saving this data")


    def savePCDandPose(self, cloud, pose):
        pose = np.array(pose)
        save_name = "data_" + str(self.save_num)
        pc = ros_numpy.numpify(cloud)
        raw_num = pc.shape[0]
        np_points = np.zeros((raw_num, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'], raw_num)
        np_points[:, 1] = np.resize(pc['y'], raw_num)
        np_points[:, 2] = np.resize(pc['z'], raw_num)
        pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
        data_g = self.hdf5_file.create_group(save_name)
        data_g.create_dataset("pose", data=pose, compression="lzf")
        data_g.create_dataset("pcl", data=pcd, compression="lzf")
        self.hdf5_file.flush()
        self.save_num += 1
        print("Saved in: %s"% save_name)

        if self.save_num >  self.test_data_num:
            print("Finish recording test dataset !!")
            self.hdf5_file.flush()
            self.hdf5_file.close()
            os._exit(10)


    def record(self):
        self.ts.registerCallback(self.callback)
        rospy.spin()


if __name__ == '__main__':

    rospy.init_node('cropped_cloud_repulisher')
    node = GenTestData("cropped_pointcloud")
    node.record()

