#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import h5py, sys, random
from tqdm import *
import open3d as o3d
import numpy as np

import rospy

data = []

hdf5_file = h5py.File("./photoneo_center_v3_optical_frame_HV8.hdf5", "r")
print("Start loading datasets !!")

pub = rospy.Publisher()

for n in tqdm(range(1, 100)):
    pcl = o3d.PointCloud()
    pcl_data = hdf5_file["data_" + str(n)]['pcl'][()]
    pcl_data = pcl_data[~np.any(np.isnan(pcl_data), axis=1)]
    pcl_data = random.sample(pcl_data, 2048)
    pcl.points = o3d.Vector3dVector(pcl_data)
    o3d.visualization.draw_geometries([pcl])
    data.append(pcl)

