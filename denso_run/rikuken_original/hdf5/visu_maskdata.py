import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np

from color_cloud_bridge.msg import out_segmentation


cnt = 0
c = 1

dataroot = "/home/ericlab/hdf5_data/original_7/"
dataset_name = "ishiyama_1000.hdf5"

datapath = dataroot + dataset_name

with h5py.File(datapath, mode="r") as f:
    pcl_data = f["data_1"]["masks"][()]
    msg_out = out_segmentation()
    for i in range(x_data.shape[0]):
        for j in range(resolution):
            msg_out.x.append(x_data[i][j][0])
            msg_out.y.append(x_data[i][j][1])
            msg_out.z.append(x_data[i][j][2])
            msg_out.instance.append(pred_new[i][j])1

