#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import argparse
import h5py

import time

# 3d show
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# ROS
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from cnn_tracking.msg import InputCNN
from cnn_tracking.msg import InputCNNArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion, TransformStamped

parser = argparse.ArgumentParser()
parser.add_argument('--division', '-d', default=50, type=int,
                    help='the number of division for each axis')

args = parser.parse_args()
div = args.division

channel = 1
axis_x = div
axis_y = div
axis_z = div
output_pos = 3
output_ori = 4


def callback(voxels_array):
    t0 = time.time()
    n_data = len(voxels_array.voxels)
    x = np.zeros((n_data, channel, axis_z, axis_y, axis_x), dtype='float32')
    y = np.zeros((n_data, 7), dtype='float32')
    print("get cnn input")

    for n in range(0, n_data):
        voxel = np.array(voxels_array.voxels[n].voxel_array.data)
        x[n, channel - 1] = voxel.reshape(axis_x, axis_y, axis_z)
        ar_z, ar_y, ar_x = np.where(x[n, channel - 1] == 1)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_aspect('equal')
# ax.voxels(x[n, channel-1], edgecolor="k")
        ax.scatter(ar_x, ar_y, ar_z)

# visualize voxel data
# point_x = []
# point_y = []
# point_z = []
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# ax.set_aspect('equal')
#
# t0 = time.time()
# for m in range(channel):
# for i in range(axis_z):
# for j in range(axis_y):
# for k in range(axis_x):
# if(voxel[(div*div*i) + (div*j) + (k)] == 1):
# x[n_data-1, m, i, j, k] = 1
#
    plt.show()

if __name__ == '__main__':
    rospy.init_node("visualize_voxel", anonymous=True)
    rospy.Subscriber("input_data", InputCNNArray, callback)
    rospy.spin()
