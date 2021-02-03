#!/usr/bin/python
# -*- coding: utf-8 -*-

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))

import rospy, rospkg, tf, random, time, sys
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped

import ros_numpy, pcl, h5py, os
from util import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from tqdm import tqdm
from tf_sync import TfMessageFilter
import message_filters


class RecordData(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.num_ = 1
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame", "photoneo_center_v3_optical_frame")
        self.p = PoseStamped()
        self.topic_name = rospy.get_param("~topic_name", "photoneo_center_v3")
        self.object_name_ = rospy.get_param("~object_name", "HV8")
        self.num_dataset = rospy.get_param("~num_dataset", 100)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        self.package_path_ = rospack.get_path("gen_dataset")
        self.save_file_path = self.package_path_ + "/../datasets/" + self.sensor_parent_frame_ + "_" + self.object_name_
        self.pcd_sub_ = message_filters.Subscriber("/" + self.topic_name + "/sensor/points", PointCloud2)
        self.sync_sub_ = message_filters.ApproximateTimeSynchronizer([self.pcd_sub_], 10, 0.01)
        self.ts_ = TfMessageFilter(self.sync_sub_, self.sensor_parent_frame_, self.object_name_, queue_size=100)
        self.init_hdf5(self.save_file_path)
        self.pcd = None

    def init_hdf5(self, file_path):
        file_path = file_path + ".hdf5"
        self.hdf5_file_ = h5py.File(file_path, 'w')

    def callback(self, point_cloud, trans_rot):
        self.receive_ok = rospy.get_param("/" + self.object_name_ + "/receive_cloud/is_ok")
        if self.receive_ok:
            rospy.set_param("/" + self.object_name_ + "/receive_cloud/is_ok", False)
            rospy.set_param("/" + self.object_name_ +  "/record_cloud/is_ok", True)
            pc = ros_numpy.numpify(point_cloud)
            height = pc.shape[0]
            width = pc.shape[1]
            np_points = np.zeros((height * width, 3), dtype=np.float32)
            np_points[:, 0] = np.resize(pc['x'], height * width)
            np_points[:, 1] = np.resize(pc['y'], height * width)
            np_points[:, 2] = np.resize(pc['z'], height * width)
            pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
            translation = np.array(trans_rot[0])
            rotation = np.array(trans_rot[1])
            pose = np.concatenate([translation, rotation])
            self.savePCDandPose(pcd, pose)
        else:
            rospy.set_param("/" + self.object_name_ +  "/record_cloud/is_ok", True)

 

    def savePCDandPose(self, cloud, pose):
        self.numpy_pose = pose
        data_g = self.hdf5_file_.create_group("data_" + str(self.num_))
        data_g.create_dataset("pose", data=pose, compression="lzf")
        data_g.create_dataset("pcl", data=cloud, compression="lzf")
        self.hdf5_file_.flush()
        self.num_ += 1
        self.bar.update(1)
        if self.num_ >  self.num_dataset:
            print("Finish recording")
            self.hdf5_file_.flush()
            self.hdf5_file_.close()
            os._exit(10)

    def record(self):
        self.ts_.registerCallback(self.callback)
        rospy.spin()


def main():
    rospy.init_node("record_data_node", anonymous=False)
    node = RecordData()

    time.sleep(5)
    node.record()

if __name__ == '__main__':
    main()

