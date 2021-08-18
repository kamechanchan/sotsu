import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1
with h5py.File('/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset//tsuchida/instance_changed_1526.hdf5', mode="r") as f:
    x_data = f["data_3"]["Points"][()]
    y_data = np.array(x_data)
    y_data[:,0] = x_data[:,0] * math.cos(math.radians(150)) - x_data[:,1] * math.sin(math.radians(150))
    y_data[:,1] = x_data[:,0] * math.sin(math.radians(150)) + x_data[:,1] * math.cos(math.radians(150))
    pcl_visu = pcl.PointCloud(y_data)
    # pcl_sama = pcl.load(pcl_visu)


    pcl.save(pcl_visu, "/home/ericlab/pointcloud/hakonaize" + "150.pcd")


    visual_after = pcl.pcl_visualization.CloudViewing()
    visual_after.ShowMonochromeCloud(pcl_visu)
    v = True
    while v:
        v = not(visual_after.WasStopped())

