#!/usr/bin/python
# -*- coding: utf-8 -*-

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
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix

parser = argparse.ArgumentParser()
parser.add_argument('--gpu', '-g', default=-1, type=int,  # default=-1
                    help='GPU ID (negative value indicates CPU)')
parser.add_argument('--division', '-d', default=50, type=int,
                    help='the number of division for each axis')
parser.add_argument('--orientation', '-o', default="rotation_matrix", type=str,  # default=-1
                    help='the number of division for each axis')
parser.add_argument('--model', '-m', default="hv7_euler2.model", type=str,  # default=-1
                    help='the number of division for each axis')
parser.add_argument('--sensor_frame_id', '-s', default="photoneo_center_optical_frame", type=str,  # default=-1
                    help='sensor frame id')

args = parser.parse_args()
gpu = args.gpu
div = args.division
orientation = args.orientation
model = args.model
sensor_frame_id = args.sensor_frame_id
if gpu >= 0:
    import cupy
xp = cupy if gpu >= 0 else np

channel = 1
axis_x = div
axis_y = div
axis_z = div
output_pos_num = 3
if orientation == "quaternion":
    output_ori_num = 4
elif orientation == "euler":
    output_ori_num = 3
elif orientation == "rotation_matrix":
    output_ori_num = 9

if orientation == "quaternion":
    import network_1 as network
elif orientation == "euler":
    import network_hv7hv8 as network
elif orientation == "rotation_matrix":
    import network_matrix as network

roslib.packages.get_pkg_dir('cnn_pose_estimator')

model_path = roslib.packages.get_pkg_dir(
    'cnn_pose_estimator') + "/scripts/result/"
model_name = model
nn = network.CNN()
if gpu >= 0:
    nn.to_gpu(0)
serializers.load_npz(model_path + model_name, nn)
print("load cnn model")


def callback(voxels_array):
    t0 = time.time()
    n_data = len(voxels_array.voxels)
    x = xp.zeros((n_data, channel, axis_z, axis_y, axis_x), dtype='float32')
    if orientation == "quaternion":
        y = xp.zeros(
            (n_data,
             output_pos_num + output_ori_num),
            dtype='float32')
    elif orientation == "euler":
        y = xp.zeros(
            (n_data,
             output_pos_num + output_ori_num),
            dtype='float32')
    elif orientation == "rotation_matrix":
        y = xp.zeros(
            (n_data,
             output_pos_num + output_ori_num),
            dtype='float32')

    print("get cnn input")
    pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=1)

    t_orientation = time.time()
    for n in range(0, n_data):
        voxel = xp.array(voxels_array.voxels[n].voxel_array.data)
        x[n, channel - 1] = voxel.reshape(axis_x, axis_y, axis_z)

# a_z,a_y,a_x = np.where(x[n,channel-1] == 1)
# ax = fig.gca(projection='3d')
# ax.set_xlim([0, div])
# ax.set_ylim([0, div])
# ax.set_zlim([0, div])
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# ax.set_aspect('equal')
# ax.scatter(a_x, a_y, a_z)
# plt.show()

# visualize voxel data
# point_x = []
# point_y = []
# point_z = []
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xla/bel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# point_x.append(5)
# point_y.append(5)
# point_z.append(5)

# t0 = time.time()
# for m in range(channel):
# for i in range(axis_z):
# for j in range(axis_y):
# for k in range(axis_x):
# if(input_data.voxel_array.data[(div*div*i) + (div*j) + (k)] == 1):
# x[n_data-1, m, i, j, k] = 1
#
# point_x.append(k)
# point_y.append(j)
# point_z.append(i)
# point_x.append(axis_x-6)
# point_y.append(axis_x-6)
# point_z.append(axis_x-6)
#
# ax.scatter(point_z, point_y, point_x)
# plt.show()

    t1 = time.time()
    y_pre = nn.predict(x)
    t_estimate = time.time()

    # 逆正規化
    y = y_pre.data
    y_pos = y[0, 0:3]
    y_ori = y[0, 3:]

    y_pos = y_pos * voxels_array.voxels[0].pose.orientation.x
    translation = Vector3(
        y_pos[0] + voxels_array.voxels[0].pose.position.x,
        y_pos[1] + voxels_array.voxels[0].pose.position.y,
     y_pos[2] + voxels_array.voxels[0].pose.position.z)

    if orientation == "quaternion":
        nrm = y_ori[0]*y_ori[0] + y_ori[1]*y_ori[1] + y_ori[2]*y_ori[2] + y_ori[3]*y_ori[3]
        y_ori_x = xp.sign(y_ori[0]) * xp.sqrt(
            (y_ori[0] * y_ori[0] / float(nrm)))
        y_ori_y = xp.sign(y_ori[1]) * xp.sqrt(
            (y_ori[1] * y_ori[1] / float(nrm)))
        y_ori_z = xp.sign(y_ori[2]) * xp.sqrt(
            (y_ori[2] * y_ori[2] / float(nrm)))
        y_ori_w = xp.sign(y_ori[3]) * xp.sqrt(
            (y_ori[3] * y_ori[3] / float(nrm)))
        rotation = Quaternion(y_ori_x, y_ori_y, y_ori_z, y_ori_w)
# print(y_pos, y_ori_x, y_ori_y, y_ori_z, y_ori_w)
    elif orientation == "euler":
        y_ori = y_ori * np.pi
        y_ori = quaternion_from_euler(y_ori[0], y_ori[1], y_ori[2])
        rotation = Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3])
        print("estimated data:{},{}".format(y_pos.round(6), y_ori.round(6)))
    elif orientation == "rotation_matrix":
        y_ori_mat = np.zeros((3, 3), dtype=np.float64)
        for i in range(0, 3):
            for j in range(0, 3):
                y_ori_mat[i][j] = y_ori[i * 3 + j]
        ##print(y_ori_mat)
        u, p = linalg.polar(y_ori_mat, side='right')
        rotation_matrix = u
        rotation_matrix = np.insert(rotation_matrix, 3, [0, 0, 0], axis=1)
        rotation_matrix = np.insert(rotation_matrix, 3, [0, 0, 0, 1], axis=0)
        ##print(rotation_matrix)
        y_ori = quaternion_from_matrix(rotation_matrix)
        rotation = Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3])

    sample_tf = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "photoneo_test_optical_frame"
    t.header.frame_id = sensor_frame_id
    t.child_frame_id = "estimated_tf"
    t.transform.translation = translation
    t.transform.rotation = rotation
    sample_tf.sendTransform(t)

    print(translation, rotation)

    t2 = time.time()
    time0 = t1 - t_orientation
    time1 = t_estimate - t1

    elapsed_time2 = t2 - t0
    print("推定時間：{}".format(elapsed_time2))

if __name__ == '__main__':
    rospy.init_node("pose_estimate", anonymous=True)
    rospy.Subscriber("input_data", InputCNNArray, callback)
    rospy.spin()
