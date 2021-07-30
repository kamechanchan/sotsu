#!/usr/bin/env python3
import h5py
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
from util_rikuken import util_rikuken
import pcl
import numpy as np
import pcl.pcl_visualization
import open3d
from color_cloud_bridge.msg import out_segmentation
import rospy

rospy.init_node("init")
pub = rospy.Publisher("otameshi_topic", out_segmentation, queue_size=10)
path = "/home/ericlab/ros_package/denso_2_ws/src/denso_branch/denso_run/rikuken_original/annotation_package/dataset/semantic_6000.hdf5"
#path = "/home/ericlab/OneDrive/DENSO/raugh_recognition/datasets/tsuchida/HV6_size_20000_range_pi_1.hdf5"
#path = util_rikuken.find_hdf5_File("/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset", "init.hdf5")
hdf5_file = h5py.File(path, "r")
loop = rospy.Rate(2)
while not rospy.is_shutdown():

    for i in range(1):
        # ff = open('/home/ericlab/tameshi_pcd/subara.txt', "w")
        poc_t = out_segmentation()
        index = 4
        pcl_data = hdf5_file["data_" + str(index)]['Points'][()]
        instance_data = hdf5_file["data_" + str(index)]['masks'][()]
        print(type(pcl_data))
        print(type(instance_data))
        print(pcl.PointCloud(pcl_data))
        list_1 = pcl_data.tolist()
        list_2 = instance_data.tolist()
        print(path)
        # print(pcl_data)
        
        print(len(list_1))
        j  =0
        for r in list_1:
            print("r_type" + "x: " + str(r[0]) + " y: " + str(r[1]) + "  z: " + str(r[2]) + "  instance" + str(list_2[i][0]))
            # ff.write(str(r[0]) + " " + str(r[1]) + " " + str(r[2]) + " " + str(list_2[i][0]) + "\n")
            poc_t.x.append(r[0])
            poc_t.y.append(r[1])
            poc_t.z.append(r[2])
            poc_t.instance.append((int)(list_2[j][0]))

            j = j + 1
        pub.publish(poc_t)
        loop.sleep()
    
    #pcd = open3d.geometry.PointCloud()
    #pcd.points = open3d.utility.Vector3dVector(pcl_data)
    #new_pcd = pcl.PointCloud(pcl_data.tolist())
    #pcl.save(new_pcd, "/home/ericlab/tameshi_pcd/" + "pcl_" + str(i+1) +".pcd")
    #open3d.write_point_cloud("/home/ericlab/tameshi_pcd/" + "pcl_" + str(i+1) +".pcd", pcd)
