import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1

dataroot = "/home/ericlab/hdf5_data/semantic_occlusion/"
dataset_name = "semantic_changed_occulution_kiriwake_9_14_1000_1.hdf5"

datapath = dataroot + dataset_name

with h5py.File(datapath, mode="r") as f:
    pcl_data = f["data_6"]["Points"][()]
    # pcl_data = pcl_data[1,:,:]
    # pcl_data = np.array(pcl_data)
    # pcl_data = pcl_data.transpose(1,0)
    print(pcl_data.shape)
    pcl_visu = pcl.PointCloud(pcl_data)
    visual_after = pcl.pcl_visualization.CloudViewing()
    visual_after.ShowMonochromeCloud(pcl_visu)
    v = True
    while v:
        v = not(visual_after.WasStopped())

