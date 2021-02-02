import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))

import numpy as np
from options.test_options import TestOptions
from dnn_test import estimation
from scipy.spatial.transform import Rotation


def pose_prediction(opt, data):
    n_data = len(data)
    row = 3
    col = n_data / row
    x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    y_pre = estimation(opt, x)
    y = np.squeeze(y_pre[0])
    y_pos = y[0:3]
    rot = Rotation.from_dcm(y[3:12].reshape(3, 3))
    #y_euler = rot.as_euler("xyz")
    y_euler = rot.as_quat()
    y = np.r_[y_pos, y_euler]

    return (y, y_pre[1])

