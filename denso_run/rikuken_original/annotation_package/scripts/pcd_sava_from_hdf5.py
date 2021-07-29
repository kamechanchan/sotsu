#!/usr/bin/env python3
import h5py
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
from util_rikuken import util_rikuken
import pcl
import numpy as np
import pcl.pcl_visualization
import open3d

path = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset/tsuchida/semantic_6000.hdf5"
#path = "/home/ericlab/OneDrive/DENSO/raugh_recognition/datasets/tsuchida/HV6_size_20000_range_pi_1.hdf5"
#path = util_rikuken.find_hdf5_File("/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset", "init.hdf5")
hdf5_file = h5py.File(path, "r")

for i in range(1):
    ff = open('/home/ericlab/tameshi_pcd/subara.txt', "w")
    pcl_data = hdf5_file["data_" + str(i + 5)]['Points'][()]
    print(type(pcl_data))
    print(pcl.PointCloud(pcl_data))
    list_1 = pcl_data.tolist()
    print(path)
    print(pcl_data)
    
    print(len(list_1))
    for r in list_1:
        ff.write(str(r) + "\n")
    
    #pcd = open3d.geometry.PointCloud()
    #pcd.points = open3d.utility.Vector3dVector(pcl_data)
    #new_pcd = pcl.PointCloud(pcl_data.tolist())
    #pcl.save(new_pcd, "/home/ericlab/tameshi_pcd/" + "pcl_" + str(i+1) +".pcd")
    #open3d.write_point_cloud("/home/ericlab/tameshi_pcd/" + "pcl_" + str(i+1) +".pcd", pcd)
