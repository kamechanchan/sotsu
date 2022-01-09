import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1

dataroot = "/home/ericlab/hdf5_data/temmat/"
dataset_name = "for_temmat_raugh_changed_HV8_size_5000_range_pi_2.hdf5"
# dataset_name = "occulution_kiriwake_11_18_1000_1.hdf5"

datapath = dataroot + dataset_name

with h5py.File(datapath, mode="r") as f:
    # pcl_data = f["data_1"]["Points"][()]
    pcl_data = f["data_1"]["pcl"][()]
    ui_data = f["data_1"]["class"][()]
    # pcl_data = pcl_data[7,:,:]
    print(pcl_data.shape)
    # pcl_data = np.array(pcl_data)
    # pcl_data = pcl_data.transpose(1,0)
    # print(pcl_data.shape)
    print(ui_data)
    pcl_visu = pcl.PointCloud(pcl_data)
    visual_after = pcl.pcl_visualization.CloudViewing()
    visual_after.ShowMonochromeCloud(pcl_visu)
    v = True
    while v:
        v = not(visual_after.WasStopped())

