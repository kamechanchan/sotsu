#!/usr/bin/python3
# -*- coding: utf-8 -*-

import h5py
import rospy
from color_cloud_bridge.msg import out_segmentation
from sklearn.cluster import KMeans
import numpy as np


cnt = 0
c = 1
print("kakunin")
rospy.init_node("progress_result")
print("tabunsouiukoto")
instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)

dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/semantic_segmentation/semantic_changed_occulution_kiriwake_9_14_1000_1.hdf5/1/"
dataset_name = "result1.hdf5"
resolution = 8192
instance_number = 25

datapath = dataroot + dataset_name
with h5py.File(datapath, mode="r") as f:
    in_data = f["data_3"]["Points"][()]
    out_data = f["data_3"]["ground_truth"][()]
    msg_out = out_segmentation()
    print("in")
    print(in_data.shape)
    print(out_data.shape)

    pred_new = np.zeros((in_data.shape[0], resolution), dtype = np.int32)
    for i in range(in_data.shape[0]):
        for j in range(resolution):
            for n in range(instance_number):
                if out_data[i, j] == 1:
                    pred_new[i, j] = 1
                else :
                    pred_new[i, j] = 2

    for i in range(in_data.shape[0]):
        # print(np.array(pred_new).shape)
        for j in range(resolution):
            msg_out.x.append(in_data[i][0][j])
            msg_out.y.append(in_data[i][1][j])
            msg_out.z.append(-in_data[i][2][j])
            msg_out.instance.append(pred_new[i][j])
            # print(type(pred_new[i][j]))
        instance_pub.publish(msg_out)
