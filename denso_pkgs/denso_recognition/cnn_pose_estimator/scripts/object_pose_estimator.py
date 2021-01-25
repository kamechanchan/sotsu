#!/usr/bin/env python
import numpy as np
from scipy import linalg

import matplotlib.pyplot as plt
import argparse
import chainer
import chainer.functions as F
import chainer.links as L

import h5py
from chainer import serializers

import chainer.computational_graph as c
import time
import math

# 3d show
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# ROS
import rospy
import roslib.packages
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from denso_recognition_msgs.msg import InputCNN
from denso_recognition_msgs.msg import InputCNNArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix


class ObjectPoseEstimator(object):
    def __init__(self, use_gpu=True, division=50, orientation="rotation_matrix", model="hv7_euler2.model", sensor_frame_id="photoneo_center_optical_frame"):
        self.use_gpu_ = use_gpu
        self.division_ = division
        self.orientation_ = orientation
        self.model_ = model
        self.sensor_frame_id_ = sensor_frame_id
        self.channel_ = 1
        self.output_pos_num_ = 3
        self.output_ori_num_ = 0
        self.axis_x_ = 0
        self.axis_y_ = 0
        self.axis_z_ = 0
        self.voxels_array_ = InputCNNArray()
        rospy.loginfo("Initialize ObjectPoseEstimater class")

    def setParameter(self):
        self.axis_x_ = self.division_
        self.axis_y_ = self.division_
        self.axis_z_ = self.division_

        if self.orientation_ == "quaternion":
            import network_1 as network
            self.output_ori_num_ = 4
        elif self.orientation_ == "euler":
            import network_hv7hv8 as network
            self.output_ori_num_ = 3
        elif self.orientation_ == "rotation_matrix":
            import network_matrix as network
            self.output_ori_num_ = 9
        else:
            self.output_ori_num_ = 0

        model_path = roslib.packages.get_pkg_dir(
            'cnn_pose_estimator') + "/scripts/result/"

        self.nn_ = network.CNN()

        if self.use_gpu_:
            import cupy
            self.xp_ = cupy
            rospy.loginfo("Use gpu")
            self.nn_.to_gpu(0)
        else:
            self.xp_ = np
            rospy.loginfo("Not use gpu")

        serializers.load_npz(model_path + self.model_, self.nn_)
        print("load cnn model")

    def estimateObjectPose(self, voxels_array):
        voxels = InputCNNArray()
        voxels = voxels_array
        estimate_pose = PoseStamped()
        estimate_pose.pose.orientation.w = 1
        t0 = time.time()
        n_data = len(voxels.voxels)

        if not n_data:
            rospy.logerr("Input empty voxel array data !!")
            return False, estimate_pose

        x = self.xp_.zeros((n_data, self.channel_, self.axis_z_,
                            self.axis_y_, self.axis_x_), dtype='float32')
        y = self.xp_.zeros((n_data, self.output_pos_num_ +
                            self.output_ori_num_), dtype='float32')

        rospy.loginfo("Get cnn input")

        t_orientation = time.time()
        for n in range(0, n_data):
            voxel = self.xp_.array(voxels.voxels[n].voxel_array.data)
            x[n, self.channel_ -
                1] = voxel.reshape(self.axis_x_, self.axis_y_, self.axis_z_)

        t1 = time.time()
        y_pre = self.nn_.predict(x)
        t_estimate = time.time()

        y = y_pre.data
        y_pos = y[0, 0:3]
        y_ori = y[0, 3:]

        y_pos = y_pos * voxels.voxels[0].pose.orientation.x
        translation = Vector3(y_pos[0] + voxels.voxels[0].pose.position.x,
                              y_pos[1] + voxels.voxels[0].pose.position.y,
                              y_pos[2] + voxels.voxels[0].pose.position.z)

        if self.orientation_ == "quaternion":
            nrm = y_ori[0]*y_ori[0] + y_ori[1]*y_ori[1] + \
                y_ori[2]*y_ori[2] + y_ori[3]*y_ori[3]
            y_ori_x = self.xp_.sign(
                y_ori[0]) * self.xp_.sqrt(y_ori[0]*y_ori[0] / float(nrm))
            y_ori_y = self.xp_.sign(
                y_ori[1]) * self.xp_.sqrt(y_ori[1]*y_ori[1] / float(nrm))
            y_ori_z = self.xp_.sign(
                y_ori[2]) * self.xp_.sqrt(y_ori[2]*y_ori[2] / float(nrm))
            y_ori_w = self.xp_.sign(
                y_ori[3]) * self.xp_.sqrt(y_ori[3]*y_ori[3] / float(nrm))
            rotation = Quaternion(y_ori_x, y_ori_y, y_ori_z, y_ori_w)

        elif self.orientation_ == "euler":
            y_ori = y_ori * np.pi
            y_ori = quaternion_from_euler(y_ori[0], y_ori[1], y_ori[2])
            rotation = Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3])
            rospy.loginfo("Estimated data:{}, {}".format(
                y_pos.round(6), y_ori.round(6)))

        elif self.orientation_ == "rotation_matrix":
            y_ori_mat = np.zeros((3, 3), dtype=np.float64)
            for i in range(0, 3):
                for j in range(0, 3):
                    y_ori_mat[i][j] = y_ori[i * 3 + j]

            u, p = linalg.polar(y_ori_mat, side='right')
            rotation_matrix = u
            rotation_matrix = np.insert(rotation_matrix, 3, [0, 0, 0], axis=1)
            rotation_matrix = np.insert(
                rotation_matrix, 3, [0, 0, 0, 1], axis=0)

            y_ori = quaternion_from_matrix(rotation_matrix)
            nrm = math.sqrt(y_ori[0]*y_ori[0] + y_ori[1] *
                            y_ori[1] + y_ori[2]*y_ori[2] + y_ori[3]*y_ori[3])
            y_ori_x = y_ori[0] / nrm
            y_ori_y = y_ori[1] / nrm
            y_ori_z = y_ori[2] / nrm
            y_ori_w = y_ori[3] / nrm
            rotation = Quaternion(y_ori_x, y_ori_y, y_ori_z, y_ori_w)

        else:
            rospy.logerr("Undefined orientation name !!")
            return False, estimate_pose

        estimate_pose.pose.position = translation
        estimate_pose.pose.orientation = rotation

        print(translation, rotation)

        t2 = time.time()
        time0 = t1 - t_orientation
        time1 = t_estimate - t1

        elapsed_time2 = t2 - t0
        print("Estimate Time:{}".format(elapsed_time2))

        return True, estimate_pose
