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
rospy.init_node("nazenanda")
print("tabunsouiukoto")
instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)

# dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/semantic_segmentation/semantic_changed_8_7_1526.hdf5/1"
dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/semantic_segmentation/semantic_changed_occulution_kiriwake_9_14_1000_1.hdf5/200/"
# dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/instance_segmentation/instance_changed_8_7_1526.hdf5+instance_changed_instance_tsuchida_8_12_500_1.hdf5+instance_changedinstance_tsuchida_8_11_1000_1.hdf5+instance_changed_instance_tsuchida_8_25_5000_1.hdf5+instance_changed_instance_tsuchida_9_2_2000_1.hdf5_epoch200/200/"
dataset_name = "result200.hdf5"
resolution = 8192
instance_number = 3
dataset_number = 3

datapath = dataroot + dataset_name
with h5py.File(datapath, mode="r") as f:
    for n in range(dataset_number):
        in_data = f["data_" + str(n)]["Points"][()]
        # out_data = f["data_" + str(n)]["ground_truth"][()]
        out_data = f["data_" + str(n)]["est"][()]
        print("in")
        print(in_data.shape)
        print(out_data.shape)
        out_data = np.array(out_data, dtype=np.uint32)

        # pred_new = np.zeros((in_data.shape[0], resolution), dtype = np.int32)
        # for i in range(in_data.shape[0]):
        #     for j in range(resolution):
        #         for n in range(instance_number):
        #             if out_data[i, j] == 1:
        #                 pred_new[i, j] = 1
        #             elif out_data[i, j] == 2:
        #                 pred_new[i, j] = 2
        #             else :
        #                 pred_new[i, j] = 3
        r = rospy.Rate(0.5)
        pred_new = []
        for i in range(in_data.shape[0]):
            # print(np.array(pred_new).shape)
            msg_out = out_segmentation()
            print(in_data[i].dtype)
            pred_new.append(out_data[i, :])
            # pred_new.append(KMeans(n_clusters=instance_number).fit_predict(out_data[i,:,:]))
            print(np.array(pred_new).shape)
            for j in range(resolution):
                msg_out.x.append(in_data[i][0][j])
                msg_out.y.append(in_data[i][1][j])
                msg_out.z.append(in_data[i][2][j])
                # msg_out.instance.append(out_data[i][j])
                # msg_out.instance.append(pred_new[i][j])
                msg_out.instance.append(pred_new[i][j])
                # print(type(pred_new[i][j]))
            # print(msg_out)
            instance_pub.publish(msg_out)
            r.sleep()
