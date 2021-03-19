#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import h5py, sys, random
from tqdm import *
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2


hdf5_file = h5py.File("../photoneo_center_v1_optical_frame_HV8.hdf5", "r")
print("Start loading datasets !!")


def publisher():
    pub = rospy.Publisher('HV8_dummy_cloud', PointCloud2, queue_size=10)
    rospy.init_node('test_node', anonymous=True)

    data_x = []
    data_y = []

    for n in tqdm(range(1, 100)):
        pcl = o3d.PointCloud()
        pcl_data = hdf5_file["data_" + str(n)]['pcl'][()]
        pose_data = hdf5_file["data_" + str(n)]['pose'][()]
        pcl_data = pcl_data[~np.any(np.isnan(pcl_data), axis=1)]
        pcl.points = o3d.Vector3dVector(pcl_data)
        o3d.visualization.draw_geometries([pcl])
        data_x.append(pcl)
        data_y.append(pose)


    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish()
        r.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass


