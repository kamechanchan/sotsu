import pcl 
import rospy
import numpy as np
import pcl.pcl_visualization
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time


cloud = pcl.load('/home/ericlab/near_1.pcd')
np_cloud = np.asarray(cloud)
pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
pcd_data = np_cloud - pcd_offset  #original
new_pcd = pcl.PointCloud(np.array(pcd_data, np.float32))
#pcl.save(new_pcd, "/home/ericlab/random_original.pcd")
pcl.save(new_pcd, '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/denso_common/denso_descriptions/object_description/meshes/PCD/random_original.pcd')