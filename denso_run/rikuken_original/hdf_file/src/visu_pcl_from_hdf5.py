import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1

dataroot = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset/tsuchida/"
dataset_name = "semantic_6000.hdf5"

datapath = dataroot + dataset_name

with h5py.File(datapath, mode="r") as f:
    pcl_data = f["data_2"]["Points"][()]
    pcl_visu = pcl.PointCloud(pcl_data)
    visual_after = pcl.pcl_visualization.CloudViewing()
    visual_after.ShowMonochromeCloud(pcl_visu)
    v = True
    while v:
        v = not(visual_after.WasStopped())

