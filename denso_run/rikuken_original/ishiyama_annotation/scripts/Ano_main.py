#!/usr/bin/env python3

from pickle import NONE
from typing_extensions import Annotated
from estimator.srv import first_input
from estimator.srv import second_input
from estimator.srv import third_input
from estimator.srv import fourth_input
from estimator.srv import five_input
from estimator.srv import six_input
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from estimator.srv import tf_quat
from estimator.srv import tf_quatRequest
import numpy as np
from std_msgs.msg import Float64MultiArray


class Client():
    def __init__(self):
        print("start")
        rospy.init_node("nice~!", anonymous=True)
        self.first_ser_name = rospy.get_param("~first_service_name", "first_input")
        self.object_name = rospy.get_param("~object_name", "second_mesh")
        self.object_count = rospy.get_param("~the_number_of_object", "25")
        self.third_ser_name = rospy.get_param("~third_service_name", "third_input")
        self.fourth_ser_name = rospy.get_param("~fourth_service_name", "fourth_input")
        self.fifth_ser_name = rospy.get_param("~fifth_service_name", "fifth_input")
        self.num_dataset = rospy.get_param("~num_dataset", "1000")
        self.six_ser_name = rospy.get_param("~six_service_name", "six_input")
        self.header_frame_id = rospy.get_param("~frame_id", "world")
        self.tf_quat_ser_name = rospy.get_param("~tf_ser_name", "quaternion")

        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.cnt = 0
        self.loop = rospy.Rate(1)
        while(int(self.num_dataset)):
            self.cnt += 1    
            self.client_main()

    def client_main(self):
        self.six_move_object()
        self.loop.sleep()

        self.first_img_pcl()
        self.second_mesh_pub()
        self.third_nearest_search()
        self.fourth_pcl_baikai()
        self.GT_tf_pub()
        self.fifth_record()
        

    def first_img_pcl(self):
        rospy.wait_for_service(self.first_ser_name)
        data = None
        try:
            start = rospy.ServiceProxy(self.first_ser_name, first_input)
            res = start(data)
            self.in_img = res.out_img
            # self.in_cloud = res.out_cloud
            self.x = res.x
            self.y = res.y
            self.z = res.z

            x_max = 0
            x_min = 0
            y_max = 0
            y_min = 0
            z_max = 0
            z_min = 0
            for j in range(np.array(self.x).shape[0]):
                if x_max > self.x[j]:
                    x_max = self.x[j]
                if y_max > self.y[j]:
                    y_max = self.y[j]
                if z_max > self.z[j]:
                    z_max = self.z[j]
                if x_min < self.x[j]:
                    x_min = self.x[j]
                if y_min < self.y[j]:
                    y_min = self.y[j]
                if z_min < self.z[j]:
                    z_min = self.z[j]
            print("hajimaruze")
            print(x_max)
            print(x_min)
            print(y_max)
            print(y_min)
            print(z_max)
            print(z_min)

            # print("self.in_cloud")
            # print(np.array(self.in_cloud.data).shape)

            # print(self.in_cloud)
            print("first_input_ok!")
        except rospy.ServiceException:
            print("service call failed first_input : ishiyama_PlaSeg")

    def second_mesh_pub(self):
        # print("kusoga")
        # print(type(self.object_count))
        # print(self.object_count)
        data = None
        self.GT_translation = []
        self.GT_rotation = []
        self.trans_list = []
        self.rot_list = []
        for i in range(int(self.object_count)):
            # print(self.object_name + "_" + str(i))
            rospy.wait_for_service(self.object_name + "_" + str(i))
            try:
                start = rospy.ServiceProxy(self.object_name + "_" + str(i), second_input)
                res = start(data)
                # print("tf_shape")
                # print(np.array(res.GT_translation).shape)
                # print(np.array(res.GT_rotation).shape)
                self.GT_translation.append(res.GT_translation)
                self.GT_rotation.append(res.GT_rotation)
                tran = Float64MultiArray(data=res.GT_translation)
                rot = Float64MultiArray(data=res.GT_rotation)
                self.trans_list += [tran]
                self.rot_list += [rot]
                # print("oafa;ljf")
                # self.in_img = res.out_img
                # self.in_cloud = res.out_cloud
                # print(self.in_cloud)
                # print("second_input_ok!:" + str(i))
            except rospy.ServiceException:
                print("service call failed second_input : mesh_bara_pub")

    def third_nearest_search(self):
        rospy.wait_for_service(self.third_ser_name)
        data = None
        try:
            start = rospy.ServiceProxy(self.third_ser_name, third_input)
            # print(type(self.in_cloud))
            # res = start(self.in_cloud)
            res = start(self.x, self.y, self.z)
            self.color_cloud = res.out_cloud
            # print(self.in_cloud)
            print("third_input_ok!")
        except rospy.ServiceException:
            print("service call failed first_input : ishiyama_PlaSeg")

    def fourth_pcl_baikai(self):
        rospy.wait_for_service(self.fourth_ser_name)
        data = None
        try:
            start = rospy.ServiceProxy(self.fourth_ser_name, fourth_input)
            # print(type(self.in_cloud))
            res = start(self.color_cloud)
            # self.x = res.x
            # self.y = res.y
            # self.z = res.z
            self.rgb = res.rgb
            self.r = res.r
            self.b = res.b
            self.g = res.g
            self.instance = res.instance
            # print(self.in_cloud)
            print("fourth_input_ok!")
        except rospy.ServiceException:
            print("service call failed fourth_input : pcl_baikai")
        
    def fifth_record(self):
        rospy.wait_for_service(self.fifth_ser_name)
        try:
            # print("fifth")
            # print(np.array(self.in_cloud).shape)
            # self.x_GT = self.in_cloud[:,0]
            # self.y_GT = self.in_cloud[:,1]
            # self.z_GT = self.in_cloud[:,2]
            start = rospy.ServiceProxy(self.fifth_ser_name, five_input)
            res = start(self.x, self.y, self.z, self.rgb, self.r, self.b, self.g, self.instance, self.cnt, self.in_img, self.trans_list, self.rot_list)
            # res = start(self.x_GT, self.y_GT, self.z_GT, self.rgb, self.r, self.b, self.g, self.instance, self.cnt, self.in_img, self.trans_list, self.rot_list)
            # print(self.in_cloud)
            print("fifth_input_ok!")
        except rospy.ServiceException:
            print("service call failed fifth_input : record_hdf5")

    def six_move_object(self):
        rospy.wait_for_service(self.six_ser_name)
        data = True
        try:
            start = rospy.ServiceProxy(self.six_ser_name, six_input)
            res = start(data)
            print("six_input_ok!")
        except rospy.ServiceException:
            print("service call failed six_input : move_object_nobox")
        self.loop.sleep()
    
    def GT_tf_pub(self):

        for i in range(self.object_count):
            GT_pose = TransformStamped()
            GT_pose.header.frame_id = self.header_frame_id
            GT_pose.child_frame_id = self.object_name + "_" + str(i) + "_GT"
            GT_pose.transform.translation.x = self.GT_translation[i][0]
            GT_pose.transform.translation.y = self.GT_translation[i][1]
            GT_pose.transform.translation.z = self.GT_translation[i][2]
            GT_pose.transform.rotation.x = self.GT_rotation[i][0]
            GT_pose.transform.rotation.y = self.GT_rotation[i][1]
            GT_pose.transform.rotation.z = self.GT_rotation[i][2]
            GT_pose.transform.rotation.w = self.GT_rotation[i][3]

            print("GT_tf_misasero")
            print(self.GT_rotation[i])
            print(self.rot_list[i].data)
            print(self.GT_translation[i])
            print(self.trans_list[i].data)

            # quat_send = tf_quatRequest()
            # quat_send.input_tf = GT_pose.transform

            # tf_quat_ser = rospy.ServiceProxy(self.tf_quat_ser_name, tf_quat)
            # change_tf = tf_quat_ser(quat_send)

            tf_pub = TransformStamped()
            tf_pub.header.frame_id = self.header_frame_id
            tf_pub.child_frame_id = self.object_name + "_" + str(i) + "_GT"
            # tf_pub.transform.rotation = GT_pose.pose.orientation
            # tf_pub.transform.translation = GT_pose.pose.position
            
            # tf_pub.transform = change_tf.output_tf
            tf_pub.transform = GT_pose.transform
            tf_pub.header.stamp = rospy.Time.now()
            self.tf.sendTransform(tf_pub)
        print("send_tf")


if __name__ == "__main__":
    start = Client()

