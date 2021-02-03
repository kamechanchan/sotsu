import sys
import os
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))

from options.test_options import TestOptions
from dnn_test import estimation
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from tf.transformations import quaternion_from_matrix
from scipy import linalg
from scipy.spatial.transform import Rotation

def pose_prediction(opt, input_data):
    resolution = 50
    x = np.array(input_data.voxel_array.data).reshape(resolution, resolution, resolution)[np.newaxis, :]
    x = x[np.newaxis, :]
    y_pre = estimation(opt, x)
    y = np.squeeze(y_pre[0])
    est_time = y_pre[1]
    y_pos = y[0:3]
    y_ori = y[3:]


    y_pos = y_pos * input_data.pose.orientation.x
    translation = Vector3(y_pos[0] + input_data.pose.position.x,
                          y_pos[1] + input_data.pose.position.y,
                          y_pos[2] + input_data.pose.position.z)



    y_ori_mat = np.zeros((3, 3), dtype=np.float64)
    for i in range(0, 3):
        for j in range(0, 3):
            y_ori_mat[i][j] = y_ori[i * 3 + j]
    rot = Rotation.from_dcm(y_ori_mat)
    y_euler = rot.as_quat()

    est_pose = PoseStamped()
    est_pose.pose.position.x = translation.x
    est_pose.pose.position.y = translation.y
    est_pose.pose.position.z = translation.z
    est_pose.pose.orientation.x = y_euler[0]
    est_pose.pose.orientation.y = y_euler[1]
    est_pose.pose.orientation.z = y_euler[2]
    est_pose.pose.orientation.w = y_euler[3]

    return (est_pose, est_time)
