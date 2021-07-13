#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..//trainer'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../gen_dataset'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../trainer/options'))

from options.test_options import TestOptions
from test import *
from cloud_util import *
import open3d as o3d
from std_msgs.msg import Header
import numpy as np
from scipy import linalg
import time, random
import pandas as pd
import rospkg


# ROS
import rospy, roslib.packages, tf2_ros
from pose_estimator_srvs.srv import PoseEstimate, PoseEstimateRequest, PoseEstimateResponse
from pose_estimator_msgs.msg import InputData
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose, PoseStamped, TransformStamped, Transform
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud



class TimeLog():
    def __init__(self, object_name):
        self.object_name = object_name
        self.file_name = self.object_name + "_" + "time_log" + ".csv"
        self.log = pd.DataFrame([], columns=["preprocess_time", "estimation_time", "refinement_time"], index=range(20))

    def set_log(self, num_array , index):
        self.log.iloc[index, 0] = num_array[0]
        self.log.iloc[index, 1] = num_array[1]
        self.log.iloc[index, 2] = num_array[2]

    def save_log(self):
        self.log.to_csv(self.file_name)


class PoseEstNode():
    def __init__(self, sub_topic_name):
        rospy.init_node("Pose_Estimation_client", anonymous=True)
        rospy.wait_for_service("pose_estimation")
        
        
        self.sub_topic_name = sub_topic_name
        
        rospack = rospkg.RosPack()
        self.start = time.time()
        self.time_file = open(rospack.get_path("estimator") + '/naka.txt', 'w')
        self.package_path = rospack.get_path("pose_estimator_measure")
        self.opt = TestOptions().parse()
        self.object_name = rospy.get_param("~object_name", "HV8")
        self.icp_flag = rospy.get_param("~icpRefine", False)
        self.arch = rospy.get_param("~arch", "PointNet_Pose")
        self.opt.arch = self.arch
        self.resolution = rospy.get_param("~resolution", 50)
        self.opt.resolution = self.resolution
        self.tf = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.sub_topic_name, PointCloud2, self.callback)
        self.pub_est = rospy.Publisher("estimated_cloud", PointCloud2, queue_size=1)
        self.pub_refine = rospy.Publisher("refine_cloud", PointCloud2, queue_size=1)
        self.df_time_log = TimeLog(self.package_path + "/result/" + self.object_name)
        self.o3d_data = None
        self.est_cloud = None
        self.ref_cloud = None
        self.homoobj = None
        self.input_data = PoseEstimateRequest()
        self.offset_data = None
        self.index = 0
        self.loop = 0
        hennsuu_difine = time.time()
        self.time_file.write("変数の定義の時間は                        : " + str(hennsuu_difine - self.start) + '秒\n')

        #self.object_name_stl = model_loader("N" + self.object_name + ".pcd")
        self.object_name_stl = model_loader('random_original.pcd')
        self.model_road = time.time()
        self.time_file.write("model_loaderの処理時間は                 : " + str(self.model_road - hennsuu_difine) + '秒\n')
        if self.object_name == "HV8":
           # self.object_name_stl.asnumpy()
            #self.object_name_stl = self.object_name_stl.scale(0.001, center=True)
            self.object_name_stl = self.object_name_stl
        self.stl_est = copy.deepcopy(self.object_name_stl)
        self.stl_ref = copy.deepcopy(self.object_name_stl)

    def callback(self, data):
        self.start_callback = time.time()
        #self.time_file.write("インスタンスからコールバックまでかかった時間は: " + str(self.start_callback - self.model_road) + '\n')

        self.o3d_data = convertCloudFromRosToOpen3d(data)
        self.open3d_time = time.time()
        self.time_file.write("点群が入力されてopen3dへ変換するまでの時間は : " + str(self.open3d_time - self.start_callback) + '秒\n')

        if self.arch == "PointNet_Pose":
            normalized_pcd, self.offset_data = getNormalizedPcd(self.o3d_data.points, 1024)
            norma = time.time()
            self.time_file.write("getNormalizedPcdの処理時間は  　　　    　: " + str(norma - self.open3d_time) + '秒\n')
            self.input_data.input_cloud = Float32MultiArray(data=np.array(normalized_pcd).flatten())

        elif self.arch == "3DCNN":
            start_time = time.time()
            np_cloud = np.asarray(self.o3d_data.points)
            pose = Pose()
            min_p, max_p, diff_max = doMinMax_inference(np_cloud)
            # min_p, max_p, diff_max = doMinMax(np_cloud)
            pose.position.x = min_p[0]
            pose.position.y = min_p[1]
            pose.position.z = min_p[2]
            pose.orientation.x = diff_max
            self.input_data.input_voxel.pose = pose

            voxel = np.zeros((self.resolution, self.resolution, self.resolution), dtype="float32")
            binary_data = runMakeVoxelBinary(np_cloud, self.resolution)
            voxel = binary_data.reshape(self.resolution, self.resolution, self.resolution)
            voxel = voxel[np.newaxis, :]
            preprocess_time = time.time() - start_time
            self.input_data.input_voxel.voxel_array = Int32MultiArray(data=np.asarray(voxel).flatten())
        elif self.arch == "JSIS3D":
            normalized_pcd, self.offset_data = getNormalizedPcd(self.o3d_data.points, 1024)
            #norma = time.time()
            #self.time_file.write("getNormalizedPcdの処理時間は  　　　    　: " + str(norma - self.open3d_time) + '秒\n')
            self.input_data.input_cloud = Float32MultiArray(data=np.array(normalized_pcd).flatten())
            #np_cloud = np.asarray(self.o3d_data.points)
            #self.input_data.input_cloud = Float32MultiArray(data=np.array(np_cloud).flatten())
        else:
            print("estimator_client.py error!: Cloud not find arch!!")
            sys.exit(3)
        
        pose_start = time.time()
        res = self.getPoseData()
        pose_finish = time.time()
        self.time_file.write("ネットワークから姿勢データを得るまでの時間は　 : " + str(pose_finish - pose_start) + '秒\n')
        est_time = res.stamp
        if self.arch == "PointNet_Pose":
            res_denormalized = self.deNormalize(res)
            deno = time.time()
            self.time_file.write("Denormalizedが終了するまでの時間は         : " + str(deno - pose_finish) + '秒\n')
            res = res_denormalized
        est_cloud = self.tfPublisher(res, self.stl_est, "estimated_tf")
        call_finish = time.time()
        self.time_file.write("tf_publisherが終了するまでの時間は         : " + str(call_finish - deno) + '秒\n')
        self.loop = self.loop + 1
        self.time_file.write(str(self.loop) + "ループ目の処理時間は    　                : " + str(call_finish - self.start_callback) + '秒\n\n\n')
      

        if self.icp_flag:
            res_refine = PoseEstimateResponse()
            res_refine.trans, ref_cloud, refine_time = self.icpRefine(est_cloud, res)
            time_log =  1000 * np.array([preprocess_time, est_time, refine_time], dtype="float32")
            self.df_time_log.set_log(time_log, self.index)
            self.index += 1
            self.df_time_log.save_log()
            self.tfPublisher(res_refine, ref_cloud,  "icpRefine")

    def tfPublisher(self, res, cloud, child_frame_id):
        header = Header()
        header.frame_id = "photoneo_center_optical_frame"
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame_id
        t.transform.translation = res.trans.transform.translation
        t.transform.rotation = res.trans.transform.rotation

        if child_frame_id == "estimated_tf":
            t.header.frame_id = "photoneo_center_optical_frame"
            ros_pcd = convertCloudFromOpen3dToRos(cloud, header)
            est_cloud = do_transform_cloud(ros_pcd, t)
            self.tf.sendTransform(t)
            print("send_transform")
            self.pub_est.publish(est_cloud)
            return est_cloud
        elif child_frame_id == "icpRefine":
            t.header.frame_id = "photoneo_center_optical_frame"
            ros_pcd = convertCloudFromOpen3dToRos(cloud, header)
            self.tf.sendTransform(t)
            self.pub_refine.publish(ros_pcd)

    def deNormalize(self, res):
        res_denormalized = PoseEstimateResponse()
        res_denormalized.trans.transform.translation.x = res.trans.transform.translation.x + self.offset_data[0]
        res_denormalized.trans.transform.translation.y = res.trans.transform.translation.y + self.offset_data[1]
        res_denormalized.trans.transform.translation.z = res.trans.transform.translation.z + self.offset_data[2]
        res_denormalized.trans.transform.rotation = res.trans.transform.rotation
        return res_denormalized

    def icpRefine(self, cloud, res):
        source_cloud = convertCloudFromRosToOpen3d(cloud)
        target_cloud = self.o3d_data
        t = Transform()
        t.translation = res.trans.transform.translation
        t.rotation = res.trans.transform.rotation

        trans_init = np.asarray([[1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])

        sensor2estimatedM = transform2homogeneousM(t)

        start_time = time.time()
        estimated2refineM = p2pICP(source_cloud, target_cloud, trans_init)
        refine_time = time.time() - start_time

        sensor2refineM = np.dot(estimated2refineM, sensor2estimatedM)
        ref_cloud = source_cloud.transform(estimated2refineM)
        pose = homogeneous2transform(sensor2refineM)
        return (pose, ref_cloud, refine_time)

    def getPoseData(self):
        try:
            client = rospy.ServiceProxy("pose_estimation", PoseEstimate)
            res = client(self.input_data)
            return res
        except rospy.ServiceException:
            rospy.loginfo("Fail.")

if __name__ == "__main__":
    try:
        node = PoseEstNode("/cloud_without_segmented")
        rospy.spin()
    except rospy.ROSInterruptException: pass
