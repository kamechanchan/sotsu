#!/usr/bin/env python3
import rospy, tf, random, time, sys
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
import ros_numpy, pcl, h5py, os 
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from tqdm import tqdm

class RecordDate(object):
    def __init__(self):
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame", "photoneo_center_optical_frame")
        self.topic_name_ = rospy.get_param("~topic_name", "/cloud_without_segmented")
        self.object_name = rospy.get_param("~object_name", "HV8")
        self.num_dataset = rospy.get_param("~num_dataset", 100)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        self.save_file_path = rospy.get_param("~save_file_path", "/home/ericlab/first.hdf5")
        self.hdf5_file = h5py.File(self.save_file_path, 'w')
        


