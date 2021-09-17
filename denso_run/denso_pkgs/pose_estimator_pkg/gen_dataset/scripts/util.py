from open3d import *
import numpy as np
from ctypes import *
import pcl

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

FIELDS_XYZRGB = FIELDS_XYZ + \
        [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)

convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromOpen3dToRos(open3d_cloud, frame_header):
    frame_header.stamp = rospy.Time.now()

    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:
        fields = FIELDS_XYZ
        cloud_data = points.astype(np.float32)
        POINTS = cloud_data.tolist()
    else:
        fields = FIELDS_XYZRGB
        colors = np.floor(np.asarray(open3d_cloud.colors)*255)
        colors = colors[:,0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        cloud_data = np.c_[points, colors]

    return pc2.create_cloud(frame_header, fields, POINTS)


def convertCloudFromRosToOpen3d(ros_cloud):
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    open3d_cloud = PointCloud()
    if len(cloud_data)==0:
        point("Converting an empty cloud")
        return None

    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3

        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float:
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data ]

        open3d_cloud.points = Vector3dVector(np.array(xyz))
        open3d_cloud.colors = Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]
        open3d_cloud.points = Vector3dVector(np.array(xyz))

    return open3d_cloud


def doMinMax(cloud):
    p = np.asarray(cloud)
    min_p = np.amin(p, axis=0)
    max_p = np.amax(p, axis=0)
    diff_max = np.amax(np.amax(p) - np.amin(p))
    return min_p, max_p, diff_max


def makeVoxel(cloud, min_p, diff_max, voxel_resolution):
    leaf = 1.0/voxel_resolution
    p = np.asarray(cloud)
    normalized_p = (p - min_p) / diff_max
    normal_p = pcl.PointCloud(normalized_p)
    sor = normal_p.make_voxel_grid_filter()
    sor.set_leaf_size(leaf, leaf, leaf)
    voxel_filter = sor.filter()
    return voxel_filter


def makeBinaryData(cloud, voxel_resolution):
    binary_voxel = np.zeros(voxel_resolution ** 3)

    for i in range(cloud.size):
        point_x = (np.asarray(cloud)[i][0] / voxel_resolution).astype(np.int32)
        point_y = (np.asarray(cloud)[i][1] / voxel_resolution).astype(np.int32)
        point_z = (np.asarray(cloud)[i][2] / voxel_resolution).astype(np.int32)
        binary_voxel[(point_z * voxel_resolution * voxel_resolution) + (point_y * voxel_resolution * voxel_resolution) + (point_x * voxel_resolution * voxel_resolution)] = 1
    return binary_voxel


def runMakeVoxelBinary(cloud, resolution=64):
    min_p, max_p, diff_max = doMinMax(cloud)
    voxel_filter = makeVoxel(cloud, min_p, diff_max, resolution)
    return makeBinaryData(voxel_filter, resolution)



def crop_geometry(data):

    print("Demo for manual geometry cropping")
    print("1) Press 'Y' twice to align geometry with positive direction of y-axis")
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")

    draw_geometries_with_editing([data])
    return data

def get_cropped(path):
    obj = read_point_cloud(path)
    return obj

