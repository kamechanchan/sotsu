#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))

import numpy as np
from options.test_options import TestOptions
from dnn_test import estimation
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped


def pose_prediction(opt, data):
    n_data = len(data)
    row = 3
    col = n_data // row
    x = np.reshape(np.array(data), (col, row))[np.newaxis, :, :]
    y_pre = estimation(opt, x)
    y = np.squeeze(y_pre[0])
    est_time = y_pre[1]
    y_pos = y[0:3]
    rot = Rotation.from_matrix(y[3:12].reshape(3, 3))
    y_euler = rot.as_quat()
    y = np.r_[y_pos, y_euler]

    est_pose = PoseStamped()
    est_pose.pose.position.x = y[0]
    est_pose.pose.position.y = y[1]
    est_pose.pose.position.z = y[2]
    est_pose.pose.orientation.x = y[3]
    est_pose.pose.orientation.y = y[4]
    est_pose.pose.orientation.z = y[5]
    est_pose.pose.orientation.w = y[6]

    return (est_pose, est_time)
