#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
from typing import final

from rospy.impl.tcpros_service import ServiceProxy
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/options'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/__init__.py'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/'))
from options.test_options import TestOptions
from models import create_model
import function as f
from cloud_util import *
from std_msgs.msg import Header
import pcl


import numpy as np
from scipy import linalg
import time

# ROS
import rospy, rospkg
import roslib.packages
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateResponse
from estimator.srv import bounding, boundingResponse
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from color_cloud_bridge.msg import out_segmentation
import ros_numpy
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class DnnNode():
    def __init__(self):
        rospy.init_node("Pose_Estimation_server")
        rospy.loginfo("Ready.")
        rospack = rospkg.RosPack()
        self.opt = TestOptions().parse()
        self.opt.dataset_model = rospy.get_param("~object_name", "HV8")
        self.opt.process_swich = rospy.get_param("~process_swich", "object_segment")
        self.opt.dataset_mode = rospy.get_param("~dataset_mode", "pose_estimation")
        self.opt.batch_size = rospy.get_param("~batch_size", 30)
        self.arch = rospy.get_param("~arch", "JSIS3D")
        self.opt.arch = self.arch
        self.opt.resolution = rospy.get_param("~resolution", 8092)
        self.opt.num_threads = rospy.get_param("~num_threads", 8)
        self.opt.gpu_id = rospy.get_param("~gpu_id", "1")
        self.package_path = rospack.get_path("estimator")
        self.checkpoints_dir = rospy.get_param("~load_path", "/home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint/onoyama/0423/PointNet/dataset_20000.hdf5/latest_net.pth")
        self.opt.checkpoints_dir = self.checkpoints_dir
        self.instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)
        self.opt.is_train = False

        # self.raugh_model = "PointNet_Pose"
        self.child_frame_id = rospy.get_param("~child_frame_id","estimated_tf")
        self.header_frame_id = rospy.get_param("~header_frame_id","photoneo_center_optical_frame")
        # self.header_frame_id = "world"
        self.tf = tf2_ros.TransformBroadcaster()
        self.object_name_stl = model_loader('random_original.pcd')
        self.stl_est = copy.deepcopy(self.object_name_stl)
        self.pub_GT = rospy.Publisher("Ground_Truth_cloud", out_segmentation, queue_size=1)
        self.pub_final = rospy.Publisher("final_raugh_estimated_cloud", PointCloud2, queue_size=1)
        # self.sub = rospy.Subscriber("/cloud_without_segmented", PointCloud2, self.maji)
        self.model = create_model(self.opt)

        self.opt.process_swich = rospy.get_param("~process_swicth2", "raugh_recognition")
        self.opt.dataset_mode = rospy.get_param("~dataset_mode2", "pose_estimation")
        self.opt.arch = rospy.get_param("~arch2", "PointNet_Pose")
        self.opt.checkpoints_dir = rospy.get_param("~load_path2", "/home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint/onoyama/0423/PointNet/dataset_20000.hdf5/latest_net.pth")
        self.raugh_model = create_model(self.opt)
        self.resolution = rospy.get_param("~resolution2", 1024)

        self.output_pos_num = 3
        self.output_ori_num = 9

    def run_service(self):
        service = rospy.Service("bounding_box", bounding, self.callback)

    # def maji(self, data):
    #     relay_data = ros_numpy.numpify(data)
    #     # data = pcl.PointCloud(np.array(relay_data, np.float32))
    #     in_data = convertCloudFromRosToOpen3d(data)

    def callback(self, req):
        self.start_callback = time.time()
        t0 = time.time()
        res = boundingResponse()
        msg_iro = out_segmentation()
        if self.arch == "PointNet_Pose":
            data = req.input_cloud.data
            est_pose, est_time = f.predict_pose(self.model, data, self.opt.resolution, "PointNet")

        elif self.arch == "3DCNN":
            data = req.input_voxel
            est_pose, est_time = f.predict_pose(self.model, data, "C3D_Voxel")

        elif self.arch == "JSIS3D":
            data = req.input_cloud.data
            # print("datadata")
            # print(type(data))
            # print(np.array(data).shape)
            segme, est_time = f.predict_pose(self.model, data, self.opt.resolution, "JSIS3D")
            self.instance_pub.publish(segme)
            # print("pubish")

        elif self.arch == "PointNet_Segmentation":
            data = np.array((req.x, req.y, req.z))
            data = data.T
            # print(type(req.))
            # data = convertCloudFromRosToOpen3d(req.cloud_in)
            # data = req.cloud_in.data
            # relay_data = ros_numpy.numpify(req.cloud_in)
            # print(type(data))
            # data = pcl.PointCloud(np.array(relay_data, np.float32))
            # print(np.array(data.points).shape)
            # print(relay_data.shape)
            # print(np.array(data).shape)
            normalized_pcd, self.offset_data = getNormalizedPcd(data, self.opt.resolution)
            segme, est_time, raugh_data = f.predict_pose(self.model, normalized_pcd, self.opt.resolution, "PointNet_Segmentation")
            # print(segme)
            self.instance_pub.publish(segme)
            # print("pubish")

            #  ***************** this is raugh part ****************
            # print(raugh_data.shape)
            final_pcd, self.offset_data = getNormalizedPcd(raugh_data, self.resolution)
            # final_pcd = final_pcd[np.newaxis, :, :]
            raugh_GT = out_segmentation()
            # print("************")
            # print(final_pcd.shape)
            for i in range(self.resolution):
                raugh_GT.x.append(final_pcd[i, 0])
                raugh_GT.y.append(final_pcd[i, 1])
                raugh_GT.z.append(final_pcd[i, 2])
            self.pub_GT.publish(raugh_GT)
            est_pose, est_time = f.predict_pose(self.raugh_model, final_pcd, self.opt.resolution, "integ_final_PointNet")
            final_est = TransformStamped()
            final_est.header.frame_id = self.header_frame_id
            final_est.child_frame_id = self.child_frame_id
            final_est.transform.translation = est_pose.pose.position
            final_est.transform.rotation = est_pose.pose.orientation
            self.tf.sendTransform(final_est)

            header = Header()
            header.frame_id = self.header_frame_id
            final_pcd = convertCloudFromOpen3dToRos(self.stl_est, header)
            est_cloud = do_transform_cloud(final_pcd, final_est)
            self.pub_final.publish(est_cloud)

            # ServiceProxy
            # client
        else:
            print("Error while pose prediction operation")

        # res = Header()
        # res.frame_id = "tsuchida"
        # res.
        res.kati = True

        # res.trans.transform.translation = est_pose.pose.position
        # res.trans.transform.rotation = est_pose.pose.orientation
        # res.trans.header.stamp = rospy.Time.now()
        # res.stamp = est_time
        call_finish = time.time()
        
        # self.loop = self.loop + 1
        # self.time_file.write(str(self.loop) + "ループ目の処理時間は    　                : " + str(call_finish - self.start_callback) + '秒\n\n\n')

        return res

if __name__ == "__main__":
    node = DnnNode()
    try:
        node.run_service()
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit()


