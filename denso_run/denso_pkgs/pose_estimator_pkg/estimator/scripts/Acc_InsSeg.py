#!/usr/bin/python3
# -*- coding: utf-8 -*-

from builtins import print
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/data'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/options'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/__init__.py'))
sys.path.append(os.path.join(os.path.dirname(__file__), './function/'))
from options.test_options import TestOptions, BaseOptions
from models import create_model
import function as f
from data import *
from dnn_test import estimation_acc
import pcl


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
from tf.transformations import projection_matrix, quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from color_cloud_bridge.msg import out_segmentation


class DnnNode():
    def __init__(self):
        rospy.init_node("Pose_Estimation_server")
        rospy.loginfo("Ready.")
        rospack = rospkg.RosPack()
        self.opt = TestOptions().parse()
        # self.opt.dataset_model = rospy.get_param("~object_name", "HV8")
        self.opt.process_swich = rospy.get_param("~process_swich", "object_segment")
        self.opt.batch_size = rospy.get_param("~batch_size", 2)
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
        self.opt.dataroot = rospy.get_param("~dataroot", "/home/ericlab/hdf5_data")
        self.dataset_model = rospy.get_param("~dataset_model", "semantic_changed_1526.hdf5")
        self.opt.dataroot_swich = rospy.get_param("~dataroot_swich", "tsuchida")
        self.opt.dataset_mode = rospy.get_param("~dataset_mode", "instance_segmentation")
        self.max_dataset_size = rospy.get_param("~max_dataset_size", "1526")
        self.instance_number = rospy.get_param("~instance_number", "7")
        self.opt.max_dataset_size = [int(self.max_dataset_size)]
        self.opt.dataset_model = [self.dataset_model]

        self.opt.is_train = False
        self.input_data = EstimateDataset(self.opt)
        self.input_data = ValDataLoader(self.input_data, self.opt)
        self.model = create_model(self.opt)

        self.output_pos_num = 3
        self.output_ori_num = 9

    def main_parts(self):
        self.start_callback = time.time()
        t0 = time.time()
        msg_iro = out_segmentation()
        timer = rospy.Rate(50)
        print(self.arch)

        total = np.zeros(self.opt.batch_size)
        tps = [[] for i in range(self.opt.batch_size)]
        fps = [[] for i in range(self.opt.batch_size)]
        est_time_all = 0

        for j, data in enumerate(self.input_data):
            x_data = data["x_data"]
            y_data = data["y_data"]
            print("fajf;af")
            print(np.array(x_data).shape)
            pcl_visu = pcl.PointCloud(x_data[0])
            pcd_dir = "/home/ericlab/data"
            pcl.save(pcl_visu, pcd_dir+"/result"+".pcd")
            segme, est_time, pred = estimation_acc(self.model, data, self.opt.resolution, self.opt.dataset_mode, self.instance_number)
            est_time_all += est_time
            print(str(j) + ":" + str(est_time))
            self.instance_pub.publish(segme)
            timer.sleep()

            #calculation each data accuracy
            proposals = [[] for i in range(x_data.shape[0])]
            for i in range(x_data.shape[0]):
                for gid in np.unique(pred[i, :]):
                    indices = (pred[i, :] == gid)
                    proposals[i] += [indices]
            instances = [[] for i in range(x_data.shape[0])]
            # print("instances:" + str(instances.shape))
            for i in range(x_data.shape[0]):
                for gid in np.unique(y_data[i, :]):
                    indices = (y_data[i, :] == gid)
                    instances[i] += [indices]
            for i in range(x_data.shape[0]):
                total[i] += len(instances[i])
                tp = np.zeros(len(proposals[i]))
                fp = np.zeros(len(proposals[i]))
                gt = np.zeros(len(instances[i]))
                for pid, u in enumerate(proposals[i]):
                    overlap = 0.0
                    detected = 0
                    for iid, v in enumerate(instances[i]):
                        iou = np.sum((u & v)) / np.sum((u | v))
                        if iou > overlap:
                            overlap = iou
                            detected = iid
                    if overlap >= 0.5:
                        tp[pid] = 1
                    else:
                        fp[pid] = 1
                tps[i] += [tp]
                fps[i] += [fp]
                # print("tensaitensaitensai")
                # print(np.array(tps).shape)
                # print(np.array(fps))

        #calculation all data average acculacy
        prediction = np.zeros(self.opt.batch_size)
        recall = np.zeros(self.opt.batch_size)
        for i in range(self.opt.batch_size):
            # print("tensaininaritai")
            # print(np.array(tps[i]).shape)
            # print(fps[i])
            tp = np.concatenate(tps[i], axis=0)
            fp = np.concatenate(fps[i], axis=0)
            # print(np.array(tp).shape)
            tp = np.sum(tp)
            fp = np.sum(fp)
            # print(tp)
            prediction[i] = tp / (tp + fp)
            recall[i] = tp / total[i]
            # print("start")
            # print("tp:" + str(tp))
            # print("fp:" + str(fp))
            # print("total:" + str(total[i]))
        mean_prediction = np.mean(prediction)
        mean_recall = np.mean(recall)
        est_time_ave = est_time_all / self.max_dataset_size

        print("results")
        print("mean_Prediction:" + str(mean_prediction))
        print("mean_Recall:" + str(mean_recall))
        print("run time:" + str(est_time_ave))

if __name__ == "__main__":
    node = DnnNode()
    eval = node.main_parts()

    print("***********finish*****************")