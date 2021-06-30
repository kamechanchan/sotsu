#!/usr/bin/env python3
import pcl 
import rospy
import numpy as np
import pcl.pcl_visualization
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time


file_name = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/pcd/save_6.pcd"
pcd = pcl.load(file_name)
#pcd = pcl.load('/home/ericlab/1_/git_file_3/gpd/tutorials/table_mug.pcd')
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowMonochromeCloud(pcd)
v = True
while v:
    v = not(visual.WasStopped())