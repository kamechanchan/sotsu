import h5py
import pcl
import pcl.pcl_visualization
import math
import numpy as np


cnt = 0
c = 1
with h5py.File('/home/ericlab/DENSO_results/August/pcl_visu/progress_output/instance_segmentation/instance_changed_8_7_1526.hdf5+instance_changed_instance_tsuchida_8_12_500_1.hdf5+instance_changedinstance_tsuchida_8_11_1000_1.hdf5/5/result5.hdf5', mode="r") as f:
    for i in f["data_1"].values():
        # cnt +=1
        # c += 1
        # if cnt == 5000:
        print(i)
            # cnt =0
    # x_data = f["data_3"]["Points"][()]
    # y_data = np.array(x_data)
    # y_data[:,0] = x_data[:,0] * math.cos(math.radians(150)) - x_data[:,1] * math.sin(math.radians(150))
    # y_data[:,1] = x_data[:,0] * math.sin(math.radians(150)) + x_data[:,1] * math.cos(math.radians(150))
    # pcl_visu = pcl.PointCloud(y_data)
    # # pcl_sama = pcl.load(pcl_visu)


    # pcl.save(pcl_visu, "/home/ericlab/pointcloud/hakonaize" + "150.pcd")


    # visual_after = pcl.pcl_visualization.CloudViewing()
    # visual_after.ShowMonochromeCloud(pcl_visu)
    # v = True
    # while v:
        # v = not(visual_after.WasStopped())

