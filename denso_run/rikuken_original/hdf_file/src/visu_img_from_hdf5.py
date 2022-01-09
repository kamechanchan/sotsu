import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np
import cv2


cnt = 0
c = 1

dataroot = "/home/ericlab/hdf5_file/"
dataset_name = "instance_tsuchida_1_1_111_1.hdf5"

datapath = dataroot + dataset_name

img_save_path = "/home/ericlab/tsuchida_cloud/tuto2.jpeg"

with h5py.File(datapath, mode="r") as f:
    img = f["data_1"]["img"][()]
    cv2.imwrite(img_save_path, img)
