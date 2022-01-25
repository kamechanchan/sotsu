#!/usr/bin/env python3
import os, sys
# from cv2 import correctMatches

# from numpy.core.arrayprint import dtype_is_implied
# from numpy.core.fromnumeric import compress
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../denso_pkgs/pose_estimator_pkg/utils/'))
from cloud_util import getNormalizedPcd_seg
from util_rikuken import util_rikuken
import h5py
from tqdm import tqdm
import rospy
import rospkg
# from sensor_msgs.msg import PointCloud2
# import ros_numpy
import numpy as np
# import pcl
# from color_cloud_bridge.msg import dummy_pcl
import datetime

# from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from estimator.srv import input_data
# from estimator.srv import five_input
# from estimator.srv import five_inputResponse
from estimator.srv import *

import cv2



class record_file():
    def __init__(self):
        rospy.init_node("recored")
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("annotation_package")
        self.directorypath = rospy.get_param("~directorypath", "/home/ericlab/ishiyama_tanomu.hdf5")
        self.filepath = rospy.get_param("~filepath", "instance")
        self.service_name = rospy.get_param("~service_name", "dummy_cloud")
        self.data_number = rospy.get_param("~data_number", 3)
        self.num_dataset = rospy.get_param("~num_dataset", 5)
        self.object_count = rospy.get_param("~object_count", 25)
        self.resolution = rospy.get_param("~resolution", 60000)
        self.getnormalized_swicth = rospy.get_param("~getnormalized_swicth", False)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        # print(type(five_input))
        #cloud_sub = rospy.Subscriber("all_cloud", PointCloud2, self.callback)
        # cloud_sub = rospy.Subscriber(self.service_name, dummy_pcl, self.callback)
        rospy.Service(self.service_name, five_input, self.callback)
        # rospy.Service(self.service_name, first_input, self.callback)
        # rospy.service("fifth_input", five_input, self.callback)
        # rospy.set_param("/is_move/ok", True)
        # rospy.set_param("/is_record/ok", False)
        # self.img_switch = rospy.get_param("~img_switch", False)
        self.init_file()
        self.num_ = 1
        self.matu = 0
        self.first = True
        self.first_count = 0
        self.ugokasu = False
        self.ugokasu_count = 0



    def init_file(self):
        util_rikuken.mkdir(self.directorypath)
        dt_now = datetime.datetime.now()
        self.all_file_path = self.directorypath + self.filepath + "_"
        time_str = str(dt_now.month) + "_" + str(dt_now.day) + "_" +  str(self.num_dataset) + "_" + str(self.data_number) +"_object_cnt_" + str(self.object_count)
        self.all_file_path = self.all_file_path + time_str + ".hdf5"
        self.hdf5_file = h5py.File(self.all_file_path, "w")
    '''
    def callback(self, cloud):
        record_ok = rospy.get_param("/is_record/ok", False)
        
        if record_ok:
            self.matu += 1
        if self.matu >= 3:
            rospy.set_param("/is_record_kekkyoku/ok", True)
            
            pc = ros_numpy.numpify(cloud)
            pcd_ls = pcl.PointCloud(np.array(pc, np.float32))
            pcl.save(pcd_ls, "/home/ericlab/tameshi_pcd/init" + str(self.num_) + ".hdf5")
           
            height = pc.shape[0]
            width = 1
            np_points = np.zeros((height*width, 4), dtype=np.float32)
            np_points[:, 0] = np.resize(pc['x'], height*width)
            np_points[:, 1] = np.resize(pc['y'], height*width)
            np_points[:, 2] = np.resize(pc['z'], height*width)
            np_points[:, 3] = np.resize(pc['rgb'], height*width)
            #pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
            
            rospy.set_param("/is_move/ok", True)
            rospy.set_param("/is_record/ok", False)
            self.matu = 0
    '''

    def callback(self, req):
        # if self.ugokasu:
        #     self.ugokasu_count = self.ugokasu_count + 1
        #     if self.ugokasu_count >= 4:
        #         rospy.set_param("/is_move/ok", True)
        #         rospy.set_param("/is_record/ok", False)
        #         self.ugokasu_count = 0
        #         self.ugokasu = False
        #     return

        # record_ok = rospy.get_param("/is_record/ok", False)    
        # move_rate = rospy.Rate(1)
        # img = rospy.Subscriber("/photoneo_center/sensor/image_color", Image, self.saveIMG)
        # print("callback:OK")
        # if self.first:
        #     if self.first_count <= 2:
        #         pass
        #     else:
        #         self.first = False
        #     self.first_count = self.first_count + 1
        #     return 
        #print("size is " + str(len(req.x)))
        # if record_ok:
        #     self.matu += 1
            # print(self.matu)
        # if self.matu >= 7:
        #     rospy.set_param("/is_record_kekkyoku/ok", True)
            #req = dummy_pcl()
        req_size = len(req.x)
        # self.all_file_path = self.all_file_path + "_" + str(req_size) + "_" + str(self.num_dataset)
        # print(req_size)
        # print(len(req.x))
        # print(len(req.instance))
        
        np_points = np.zeros((req_size, 4), dtype=np.float32)
        # np_points = np.zeros((req_size, 3), dtype=np.float32)
        # np_masks = np.zeros((req_size, 1), dtype=np.float32)
        # x_max = 0
        # x_min = 0
        # y_max = 0
        # y_min = 0
        # z_max = 0
        # z_min = 0
        for i in range(req_size):
            np_points[i, 0] = req.x[i]
            np_points[i, 1] = req.y[i]
            np_points[i, 2] = req.z[i]
            np_points[i, 3] = req.instance[i]
            # np_masks[i, 0] = req.instance[i]
            # if x_max > req.x[i]:
            #     x_max = req.x[i]
            # if y_max > req.y[i]:
            #     y_max = req.y[i]
            # if z_max > req.z[i]:
            #     z_max = req.z[i]
            # if x_min < req.x[i]:
            #     x_min = req.x[i]
            # if y_min < req.y[i]:
            #     y_min = req.y[i]
            # if z_min < req.z[i]:
            #     z_min = req.z[i]
        #     if x_max > np_points[i,0]:
        #         x_max = np_points[i,0]
        #     if y_max > np_points[i,1]:
        #         y_max = np_points[i,1]
        #     if z_max > np_points[i,2]:
        #         z_max = np_points[i,2]
        #     if x_min < np_points[i,0]:
        #         x_min = np_points[i,0]
        #     if y_min < np_points[i,1]:
        #         y_min = np_points[i,1]
        #     if z_min < np_points[i,2]:
        #         z_min = np_points[i,2]

        if self.getnormalized_swicth == True:
            np_points, offset = getNormalizedPcd_seg(np_points, self.resolution)
            np_points[:,:3] += offset
        print("save_points.shape")
        print(np_points.shape)
        # print("hajimaruze")
        # print(req_size)
        # print(x_max)
        # print(x_min)
        # print(y_max)
        # print(y_min)
        # print(z_max)
        # print(z_min)
        # print("OKOKOKOK")
        # rospy.wait_for_service("input_img")
        # print("kokoda")
        # try:
        # data = None
        # start = rospy.ServiceProxy("input_img", input_data)
        # res = start(data)
        img = req.img
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img, "bgr8")
        # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        self.translation = []
        self.rotation = []
        for i in range(int(self.object_count)):
            self.translation += [req.GT_translation[i].data]
            self.rotation += [req.GT_rotation[i].data]
        # self.pose = np.concatenate(translation, rotation)
        # print("self.pose")
        # print(self.pose.shape)

        # print(img)
        # self.savePCD(np_points, np_masks, img, req.i)
        self.savePCD(np_points, img, req.i)
        # self.savePCD(pcl, img, req.i)
        res = five_inputResponse()
        res.no = True
        return res
            # self.ugokasu = True
            # self.matu = 0
            # except rospy.ServiceException:
            #     print("service call failed input_data")
            #for i in range(10):
                #print(str(req.r[i]) + " " + str(req.g[i]) + " " + str(req.b[i]) + " " + str(req.instance[i]))
            
            # print(img)
            # self.savePCD(np_points, np_masks, img)
            # self.ugokasu = True
            # matu_loop = rospy.Rate(2)
            # count = 0
            # while count <= 0:
            #     matu_loop.sleep()
            #     count = count + 1

            
            
            # self.matu = 0

            

    # def savePCD(self, cloud, masks, img, index):
    def savePCD(self, cloud, img, index):
        # if self.num_ >  self.num_dataset:
        #     rospy.signal_shutdown('finish')
        #     os._exit(10)
        # else:
            # print("data_" + str(self.num_))
        data_g = self.hdf5_file.create_group("data_" + str(index))
        data_g.create_dataset("Points", data=cloud, compression="lzf")
        # data_g.create_dataset("masks", data=masks, compression="lzf")
        # print("img")
        # print(img)
        # print(img.shape)
        # print(img.dtype)
        # print("rotation")
        # print(type(self.rotation))
        # print(type(self.rotation[0]))
        # print("gatika")
        # print(self.rotation)
        # print(self.translation)
        data_g.create_dataset("img", data=img, compression="lzf")
        data_g.create_dataset("translation", data=self.translation, compression="lzf")
        data_g.create_dataset("rotation", data=self.rotation, compression="lzf")
        self.hdf5_file.flush()
        # index += 1
        self.bar.update(1)
        if index == self.num_dataset:
            print("Finish recording")
            print("save on" + self.all_file_path)
            self.hdf5_file.flush()
            self.hdf5_file.close()

    # def saveIMG(self, img):
    #     bridge = CvBridge()
    #     out_img = bridge.imgreq_to_cv2(img, "bgr8")
    #     return out_img
        

if __name__=='__main__':
    ok = record_file()
    while not rospy.is_shutdown():
        rospy.spin()


        









