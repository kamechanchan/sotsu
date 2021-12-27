from __future__ import print_function
import sys
import os

from numpy.lib.function_base import select

from geometry_msgs import msg
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))

import numpy as np
from options.test_options import TestOptions
from dnn_test import estimation
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import KMeans
from color_cloud_bridge.msg import out_segmentation


def pose_prediction(opt, data, resolution, arg):
    n_data = len(data)
    row = 3
    col = n_data // row
    cluster_number = 25
    print("shape******************: " + str(np.array(data).shape))
    x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    print("shpae" + str(x.shape))
    y_pre = estimation(opt, x)
    print("y_data")
    print(y_pre[0].shape)
    y = np.squeeze(y_pre[0])
    est_time = y_pre[1]
    y = KMeans(n_clusters=cluster_number).fit_predict(y)
    print("KMEAns_kme*****************")
    print(y.shape)
    # print(type(y))
    msg_out = out_segmentation()
    for i in range(resolution):
        msg_out.x.append(x[0][i][0])
        msg_out.y.append(x[0][i][1])
        msg_out.z.append(x[0][i][2])
        msg_out.instance.append(y[i])

    if arg == "segment_by_JSIS3D":
        x = np.squeeze(x)
        y = y[:,np.newaxis]
        # print("x")
        # print(x.shape)
        # print("y")
        # print(y.shape)
        inout_concat_data = np.hstack([x, y])
        # print("inout")
        # print(inout_concat_data.shape)
        raugh_data = []
        big_data = 0
        cnt_list = [0]*cluster_number
        for i in range(resolution):
            for j in range(cluster_number):
                if inout_concat_data[i,3] == j:
                    cnt_list[j] += 1
        
        sort_list = sorted(cnt_list, reverse=True)
        for i in range(cluster_number):
            if sort_list[2] == cnt_list[i]:
                select_object = i
        for i in range(resolution):
            if inout_concat_data[i,3] == select_object:
                raugh_data.append(inout_concat_data[i,:])
        print("raugh_data")
        print(np.array(raugh_data).shape)
        return (msg_out, est_time, raugh_data)

    return (msg_out, est_time)