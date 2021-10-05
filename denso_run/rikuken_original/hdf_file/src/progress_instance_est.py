#!/usr/bin/python3
# -*- coding: utf-8 -*-

import h5py
from torch.nn.functional import prelu
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

dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/instance_segmentation/instance_changed_8_7_1526.hdf5+instance_changed_instance_tsuchida_8_12_500_1.hdf5+instance_changedinstance_tsuchida_8_11_1000_1.hdf5+instance_changed_instance_tsuchida_8_25_5000_1.hdf5+instance_changed_instance_tsuchida_9_2_2000_1.hdf5_epoch200/200/"
dataset_name = "result200.hdf5"
resolution = 8192
instance_number = 25
dataset_number = 2

datapath = dataroot + dataset_name
with h5py.File(datapath, mode="r") as f:
    for n in range(dataset_number):
        in_data = f["data_" + str(n)]["Points"][()]
        out_data = f["data_" + str(n)]["est"][()]
        # msg_out = out_segmentation()
        print(in_data.shape)
        print(out_data.shape)
        pred_new = []
        print(n)
        # print(in_data.shape)
        # print(out_data.shape)
        for i in range(in_data.shape[0]):
            msg_out = out_segmentation()
            pred_new.append(KMeans(n_clusters=instance_number).fit_predict(out_data[i,:,:]))
            print(np.array(pred_new).shape)
            for j in range(resolution):
                msg_out.x.append(in_data[i][0][j])
                msg_out.y.append(in_data[i][1][j])
                msg_out.z.append(in_data[i][2][j])
                msg_out.instance.append(pred_new[i][j])
                # print(type(pred_new[i][j]))
            # print(msg_out)
            instance_pub.publish(msg_out)