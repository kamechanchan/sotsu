import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1

dataroot = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/instance_segmentation/instance_changed_8_7_1526.hdf5+instance_changed_instance_tsuchida_8_12_500_1.hdf5+instance_changedinstance_tsuchida_8_11_1000_1.hdf5+instance_changed_instance_tsuchida_8_25_5000_1.hdf5+instance_changed_instance_tsuchida_9_2_2000_1.hdf5_epoch200/200/"
dataset_name = "result200.hdf5"

datapath = dataroot + dataset_name

with h5py.File(datapath, mode="r") as f:
    pcl_data = f["data_1"]["Points"][()]
    pcl_data = pcl_data[7,:,:]
    pcl_data = np.array(pcl_data)
    pcl_data = pcl_data.transpose(1,0)
    print(pcl_data.shape)
    pcl_visu = pcl.PointCloud(pcl_data)
    visual_after = pcl.pcl_visualization.CloudViewing()
    visual_after.ShowMonochromeCloud(pcl_visu)
    v = True
    while v:
        v = not(visual_after.WasStopped())

