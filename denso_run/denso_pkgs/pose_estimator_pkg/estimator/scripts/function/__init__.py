#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '.'))

from geometry_msgs.msg import TransformStamped


from pointnet_est import pose_prediction as pp1
from voxel_est import pose_prediction as pp2
from JSIS3D_est import pose_prediction as pp3
from color_cloud_bridge.msg import out_segmentation

def num2ros_transform(pos, ori):
    trans = TransformStamped()
    trans.transform.translation.x = pos[0]
    trans.transform.translation.y = pos[1]
    trans.transform.translation.z = pos[2]
    trans.transform.rotation.x = ori[0]
    trans.transform.rotation.y = ori[1]
    trans.transform.rotation.z = ori[2]
    trans.transform.rotation.w = ori[3]
    return trans

def predict_pose(model, data, arg):
    if arg == "PointNet":
        y, est_time = pp1(model, data)
        return y, est_time

    elif arg == "C3D_Voxel":
        y, est_time = pp2(model, data)
        return y, est_time
    
    elif arg == "JSIS3D":
        segment_out, est_time = pp3(model, data)
        return segment_out, est_time
    
    else:
        print("Cloud not predict DNN arch. __init_.py Error!")

