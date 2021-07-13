#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))

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
from utils import util
import pose_estimator_srvs
from pose_estimator_srvs.srv import range1, range1Request, range1Response



class RecordData(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.num_ = 1
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame", "photoneo_center_optical_frame")
        self.p = PoseStamped()
        self.topic_name = rospy.get_param("~topic_name", "photoneo_center")
        self.object_name_ = rospy.get_param("~object_name", "HV8")

        self.num_dataset = rospy.get_param("~num_dataset", 20000)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        self.package_path_ = rospack.get_path("gen_dataset")
        self.save_file_path = rospy.get_param("~save_directory", "/home/ericlab/")
        self.angle_range = rospy.get_param("~angle_range", 2)

        self.pcd_sub_ = message_filters.Subscriber("/cloud_without_segmented", PointCloud2)
        self.sync_sub_ = message_filters.ApproximateTimeSynchronizer([self.pcd_sub_], 10, 0.01)
        self.ts_ = TfMessageFilter(self.sync_sub_, self.sensor_parent_frame_, self.object_name_, queue_size=100)
        self.init_hdf5(self.save_file_path)
        self.pcd = None
        rospy.set_param("/shori_1", 0)
        rospy.set_param("/shori_2", 0)
        rospy.set_param("/shori_3", 0)
        rospy.set_param("/shori_4", 0)

    def init_hdf5(self, file_path):
        util.mkdir(file_path)
        file_path = file_path + "/" + self.object_name_ + "_size_" + str(self.num_dataset) + "_range_pi_" + str(self.angle_range) +".hdf5"
        self.hdf5_file_ = h5py.File(file_path, 'w')
        self.all_file_path = file_path

    def callback(self, point_cloud, trans_rot):
        get_shori_3 = rospy.get_param("/shori_3")
        get_shori_3 = get_shori_3 + 1
        print("my_callback_tf_pc is " + str(get_shori_3))
        get_shori_4 = rospy.get_param("/shori_4")
        print("signal_message_kakuin" + str(get_shori_4))
        get_shori_2 = rospy.get_param("/shori_1")
        print("input_callblack_kakunin" + str(get_shori_2))
        rospy.set_param("/shori_3", get_shori_3)
        self.receive_ok = rospy.get_param("/" + self.object_name_ + "/receive_cloud/is_ok")
        
        if self.receive_ok:
            
            rospy.set_param("/" + self.object_name_ + "/receive_cloud/is_ok", False)
            rospy.set_param("/" + self.object_name_ +  "/record_cloud/is_ok", False)
            pc = ros_numpy.numpify(point_cloud)
            height = pc.shape[0]
            width = 1
            np_points = np.zeros((height * width, 3), dtype=np.float32)
            np_points[:, 0] = np.resize(pc['x'], height * width)
            np_points[:, 1] = np.resize(pc['y'], height * width)
            np_points[:, 2] = np.resize(pc['z'], height * width)
            pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
            translation = np.array(trans_rot[0])
            rotation = np.array(trans_rot[1])
            #if true:
                #f = open('/home/ericlabshinya/dataset_pose.txt', 'w')
                #f.writelines(str(translation))
                #f.writelines(str(rotation))
                #f.close()
                #new_pcd = pcl.PointCloud(np.array(pcd, np.float32))
                #pcl.save(new_pcd, "/home/ericlabshinya/random_1.pcd")

            pose = np.concatenate([translation, rotation])
            self.savePCDandPose(pcd, pose)
        else:
            rospy.set_param("/" + self.object_name_ +  "/record_cloud/is_ok", True)
    
    def tf_lookup(self, source_frame, target_frame):
        lister = tf.TransformListener()
        while 1:
            try:
                (trans, rot) = lister.lookupTransform(source_frame, target_frame, rospy.Time(0))
                #f1 = open('/home/ericlab/groud_truth_pose.txt', 'w')
                #f1.writelines(str(trans))
                #f1.writelines(str(rot))
                #f1.close()

                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        
        

 

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
            print("save on" + self.all_file_path)
            self.hdf5_file_.flush()
            self.hdf5_file_.close()
            rospy.signal_shutdown('finish')
            os._exit(10)


    def record(self):
        while not rospy.is_shutdown():
            self.ts_.registerCallback(self.callback)
            rospy.spin()



def main():
    rospy.init_node("record_data_node", anonymous=False)
    node = RecordData()
    #node.tf_lookup('/photoneo_center_optical_frame', '/HV8')

    time.sleep(5)
    node.record()

if __name__ == '__main__':
    main()

