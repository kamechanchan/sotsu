import pcl 
import rospy
import numpy as np
import pcl.pcl_visualization
import open3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time

def create_pcl(width, height, depth, dense):
    t1 = time.time()
    np_cloud = np.zeros((dense,  3), float)
    width_ele = np.arange(-width/2, width/2, 0.01, float) 
    height_ele = np.arange(-height/2, height/2, 0.01, float)
    depse_ele = np.arange(-depth/2, depth/2, 0.01, float)
    
    #print(np.random.choice(width_ele.flatten()))
    for i in range(dense):
        np_cloud[i][0] = np.random.choice(width_ele.flatten())
        np_cloud[i][1] = np.random.choice(height_ele.flatten())
        np_cloud[i][2] = np.random.choice(depse_ele.flatten())
    t2 = time.time()
    print("処理時間は" + str(t2-t1))
    #visualize_open3d(np_cloud)
    visualize_pcl(np_cloud)
    
    

def visualize_open3d(np_cloud):
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(np_cloud)
    open3d.visualization.draw_geometries([pcd])

def visualize_pcl(np_cloud):
    pcd = pcl.PointCloud(np.array(np_cloud, np.float32))
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(pcd)
    v = True
    while v:
        v = not(visual.WasStopped())

'''
def read_ros():
    for p in pc2.read_points(data, skip_nans=False):
'''

create_pcl(100, 50, 10, 100000)
rospy.wait_for_message("2", sensor_msgs.PointCloud, rospy.Duration(100))
    