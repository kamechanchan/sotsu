import pcl 
import rospy
import numpy as np
import pcl.pcl_visualization
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time


<<<<<<< HEAD
pcd = pcl.load('/home/tsuchidashinya/near_1.pcd')
=======
pcd = pcl.load('/home/tsuchidashinya/1_/git_file_3/gpd/tutorials/table_mug.pcd')
>>>>>>> e6cc65836113879060692d92bd2bc7d3953d678e
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowMonochromeCloud(pcd)
v = True
while v:
    v = not(visual.WasStopped())