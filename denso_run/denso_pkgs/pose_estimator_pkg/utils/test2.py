from cloud_util import *
import open3d as o3d
import pcl

cloud = o3d.read_point_cloud("bunny.pcd")
min_p, max_p, diff_max = doMinMax(cloud.points)
binary = runMakeVoxelBinary(cloud.points)
