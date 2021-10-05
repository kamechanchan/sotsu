import sys
import os

from geometry_msgs import msg
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))

import numpy as np
from options.test_options import TestOptions
from dnn_test import estimation
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import KMeans
from color_cloud_bridge.msg import out_segmentation


def pose_prediction(opt, data, resolution):
    n_data = len(data)
    row = 3
    col = n_data // row
    # print("majika**********")
    # print(np.array(data).shape)
    x = data[np.newaxis, :, :]
    # print(np.array(x).shape)
    # x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    # print("shpae" + str(x.shape[0]))
    y_pre = estimation(opt, x)
    # print("y_data")
    # print(y_pre[0].shape)
    y = np.squeeze(y_pre[0])
    # print("tanomuze")
    # print(y)
    # print(y.shape)
    est_time = y_pre[1]
    msg_out = out_segmentation()
    raugh_data = []
    for i in range(resolution):
        msg_out.x.append(x[0][i][0])
        msg_out.y.append(x[0][i][1])
        msg_out.z.append(x[0][i][2])
        msg_out.instance.append(y[i])
        # print(y[i])
        if y[i] == 0:
            raugh_data.append(x[0, i, :])
            # print("*************")
    raugh_data = np.array(raugh_data)
    # raugh_data = raugh_data[np.newaxis, : , :]

    return (msg_out, est_time, raugh_data)
