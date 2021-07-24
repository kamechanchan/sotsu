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


def pose_prediction(opt, data):
    n_data = len(data)
    row = 3
    col = n_data // row
    x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    print("shpae" + str(x.shape))
    y_pre = estimation(opt, x)
    y = np.squeeze(y_pre[0])
    est_time = y_pre[1]
    y = KMeans(n_clusters=7).fit_predict(y)
    msg_out = out_segmentation()
    for i in range(1024):
        msg_out.x.append(x[0][i][0])
        msg_out.y.append(x[0][i][1])
        msg_out.z.append(x[0][i][2])
        msg_out.instance.append(y[i])

    return (msg_out, est_time)
