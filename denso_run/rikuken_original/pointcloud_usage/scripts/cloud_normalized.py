import pcl 
import rospy
import numpy as np
import pcl.pcl_visualization
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import h5py

path = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset/init.hdf5"
hdf5_file = h5py.File(path, "r")
pcl_data = hdf5_file["data_" + str(1)]['pcl']
new_pcd = np.array(pcl_data, np.float32)
print(new_pcd)
ne = pcl.PointCloud(np.array(new_pcd, np.float32))
#new_pcd = pcl.PointCloud(pcl_data)
#pcl.save(pcl_data, "/home/ericlab/tameshi_pcd/" + "pcl_" + str(i+1) +".pcd")

#cloud = pcl.load('//home/ericlab/ah.pcd')
#np_cloud = np.asarray(cloud)
#pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
#pcd_data = np_cloud - pcd_offset  #original
#new_pcd = pcl.PointCloud(np.array(pcd_data, np.float32))
#pcl.save(new_pcd, "/home/ericlabshinya/random_original.pcd")
#pcl.save(new_pcd, '/home/ericlab/tameshi_pcd/yah.pcd')