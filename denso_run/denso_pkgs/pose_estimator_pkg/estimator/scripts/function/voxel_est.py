import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../trainer'))

from options.test_options import TestOptions
from dnn_test import estimation


def pose_prediction(opt, data):
    orientation = "quaternion"
    n_data = len(input_voxel)
    x = np.zeros((n_data, channel, axis_z, axis_y, axis_x), dtype="float32")

    for n in range(0, n_data):
        voxel = np.array(input_voxel.voxels[n].input_voxel.data)
        x[n, channel - 1] = voxel.reshape(axis_x, axis_y, axis_z)

    y_pre = estimation(self.opt, x)
    y = y_pre[0].squeeze()
    y_pos = y[0, 0:3]
    y_ori = y[0, 3:]

    y_pos = y_pos * input_data.voxels[0].pose.orientation.x
    translation = Vector3(y_pos[0] + input_data.voxels[0].pose.position.x,
                          y_pos[1] + input_data.voxels[0].pose.position.y,
                          y_pos[2] + input_data.voxels[0].pose.position.z)

    if orientation == "quaternion":
        nrm = y_ori[0]*y_ori[0] + y_ori[1] + y_ori[2]*y_ori[2] + y_ori[3]*y_ori[3]

        y_ori_x = xp.sign(y_ori[0]) * xp.sqrt(
        (y_ori[0] * y_ori[0] / float(nrm)))
        y_ori_y = xp.sign(y_ori[1]) * xp.sqrt(
        (y_ori[1] * y_ori[1] / float(nrm)))
        y_ori_z = xp.sign(y_ori[2]) * xp.sqrt(
        (y_ori[2] * y_ori[2] / float(nrm)))
        y_ori_w = xp.sign(y_ori[3]) * xp.sqrt(
        (y_ori[3] * y_ori[3] / float(nrm)))
        rotation = Quaternion(y_ori_x, y_ori_y, y_ori_z, y_ori_w)

    elif orientation == "euler":
        y_ori = y_ori * np.pi
        y_ori = quaternion_from_euler(y_ori[0], y_ori[1], y_ori[2])
        rotation = Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3])

    elif orientation == "rotation_matrix":
        y_ori_mat = np.zeros((3, 3), dtype=np.float64)
        for i in range(0, 3):
            y_ori_mat[i][j] = y_ori[i * 3 + j]
        u, p = linalg.polar(y_ori_mat, side="right")
        rotation_matrix = u
        rotation_matrix = np.insert(rotation_matrix, 3, [0, 0, 0], axis=1)
        rotation_matrix = np.insert(rotation_matrix, 3, [0, 0, 0, 1], axis=0)
        y_ori = quaternion_from_matrix(rotation_matrix)
        rotation = Quaternion(y_ori[0], y_ori[1], y_ori[2], y_ori[3])

    return y_pos, rotation, y_pre[1]
