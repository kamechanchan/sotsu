#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/options'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/__init__.py'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/'))
from options.test_options import TestOptions
from models import create_model
from dnn_test import estimation
import function as f


import numpy as np
from scipy import linalg
import time

# ROS
import rospy, rospkg
import roslib.packages
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateResponse
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion, TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from color_cloud_bridge.msg import out_segmentation


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
        self.arch_raugh = rospy.get_param("~arch_second", "PointNet_Pose")
        self.opt.arch = self.arch
        self.opt.resolution = rospy.get_param("~resolution", 8092)
        self.opt.num_threads = rospy.get_param("~num_threads", 8)
        self.opt.gpu_id = rospy.get_param("~gpu_id", "1")
        self.package_path = rospack.get_path("estimator")
        self.checkpoints_dir = rospy.get_param("~load_path", "/home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint/onoyama/0423/PointNet/dataset_20000.hdf5/latest_net.pth")
        self.opt.checkpoints_dir = self.checkpoints_dir
        self.checkpoints_dir_raugh = rospy.get_param("~load_path_second", "/home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint/onoyama/0423/PointNet/dataset_20000.hdf5/latest_net.pth")
        self.instance_pub = rospy.Publisher("instance_pub", out_segmentation, queue_size=10)
        self.process_num = rospy.get_param("~process_num", "single")
        self.opt.is_train = False

        if self.process_num == "single":
            self.model = create_model(self.opt)
        elif self.process_num == "double":
            self.opt.arch = self.arch
            self.opt.checkpoints_dir = self.checkpoints_dir
            self.model_seg = create_model(self.opt)
            self.opt.arch = self.arch_raugh
            self.opt.checkpoints_dir = self.checkpoints_dir_raugh
            self.model_raugh = create_model(self.opt)

        self.output_pos_num = 3
        self.output_ori_num = 9

    def run_service(self):
        service = rospy.Service("pose_estimation", PoseEstimate, self.callback)

    def callback(self, req):
        self.start_callback = time.time()
        t0 = time.time()
        res = PoseEstimateResponse()
        msg_iro = out_segmentation()
        print(self.arch)
        print("hai")
        if self.process_num == "single":
            if self.arch == "PointNet_Pose":
                data = req.input_cloud.data
                est_pose, est_time = f.predict_pose(self.model, data, self.opt.resolution, "PointNet")
                res.success = True

            elif self.arch == "3DCNN":
                data = req.input_voxel
                est_pose, est_time = f.predict_pose(self.model, data, "C3D_Voxel")
                res.success = True

            elif self.arch == "JSIS3D":
                data = req.input_cloud.data
                print("datadata")
                print(type(data))
                print(np.array(data).shape)
                segme, est_time = f.predict_pose(self.model, data, self.opt.resolution, "JSIS3D")
                res.success = True
                self.instance_pub.publish(segme)
                print("pubish")

            elif self.arch == "PointNet_Segmentation":
                print("ithi")
                data = req.input_cloud.data
                segme, est_time = f.predict_pose(self.model, data, self.opt.resolution, "PointNet_Segmentation")
                res.success = True
                self.instance_pub.publish(segme)
                print("pubish")

            else:
                print("Error while pose prediction operation")
                res.success = False

        if self.process_num == "double":
            data = req.input_cloud.data
            segme, est_time, raugh_data = f.predict_pose(self.model_seg, data, self.opt.resolution, "segment_by_JSIS3D")
            res.success = True
            self.instance_pub.publish(segme)
            est_pose, est_time = f.predict_pose(self.model_raugh, raugh_data, self.opt.resolution, "integ_PointNet")
            res.success = True
        else:
            print("Error process_num!")
            res.success = False

        res.trans.transform.translation = est_pose.pose.position
        res.trans.transform.rotation = est_pose.pose.orientation
        res.trans.header.stamp = rospy.Time.now()
        res.stamp = est_time
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


