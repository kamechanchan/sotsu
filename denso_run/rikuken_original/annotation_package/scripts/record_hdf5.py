#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
from util_rikuken import util_rikuken
import h5py
from tqdm import tqdm
import rospy
import rospkg
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import pcl
from color_cloud_bridge.msg import dummy_pcl



class record_file(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("annotation_package")
        self.directorypath = rospy.get_param("~directorypath", "/home/ericlab/ishiyama_tanomu.hdf5")
        self.filepath = rospy.get_param("~filepath", "ishiyama_1000.hdf5")
        self.sub_topic_name = rospy.get_param("~sub_topic_name", "dummy_cloud")
        self.num_dataset = rospy.get_param("~num_dataset", 5)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        #cloud_sub = rospy.Subscriber("all_cloud", PointCloud2, self.callback)
        cloud_sub = rospy.Subscriber(self.sub_topic_name, dummy_pcl, self.callback)
        rospy.set_param("/is_move/ok", True)
        self.init_file()
        self.num_ = 1
        self.matu = 0
        

    def init_file(self):
        util_rikuken.mkdir(self.directorypath)
        self.all_file_path = self.directorypath + self.filepath
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

    def callback(self, msg):
        record_ok = rospy.get_param("/is_record/ok", False)    
        #print("size is " + str(len(msg.x)))
        if record_ok:
            self.matu += 1
        if self.matu >= 4:
            rospy.set_param("/is_record_kekkyoku/ok", True)
            #msg = dummy_pcl()
            msg_size = len(msg.x)
            # print(msg_size)
            # print(len(msg.x))
            # print(len(msg.instance))
           
            np_points = np.zeros((msg_size, 3), dtype=np.float32)
            np_masks = np.zeros((msg_size, 1), dtype=np.float32)
            for i in range(msg_size):
                np_points[i, 0] = msg.x[i]
                np_points[i, 1] = msg.y[i]
                np_points[i, 2] = msg.z[i]
                np_masks[i, 0] = msg.instance[i]
            
            #for i in range(10):
                #print(str(msg.r[i]) + " " + str(msg.g[i]) + " " + str(msg.b[i]) + " " + str(msg.instance[i]))
            '''
            np_points[:, 0] = np.resize(msg.x, msg_size)
            np_points[:, 1] = np.resize(msg.y, msg_size)
            np_points[:, 2] = np.resize(msg.z, msg_size)
            np_points[:, 3] = np.resize(msg.rgb, msg_size)
            '''
            self.savePCD(np_points, np_masks)
            rospy.set_param("/is_move/ok", True)
            rospy.set_param("/is_record/ok", False)
            self.matu = 0

            

    def savePCD(self, cloud, masks):
        if self.num_ >  self.num_dataset:
            rospy.signal_shutdown('finish')
            os._exit(10)
        else:
            
            data_g = self.hdf5_file.create_group("data_" + str(self.num_))
            data_g.create_dataset("Points", data=cloud, compression="lzf")
            data_g.create_dataset("masks", data=masks, compression="lzf")
            self.hdf5_file.flush()
            
            self.num_ += 1
            self.bar.update(1)
            if self.num_ > self.num_dataset:
                print("Finish recording")
                print("save on" + self.all_file_path)
                self.hdf5_file.flush()
                self.hdf5_file.close()
        

if __name__=='__main__':
    rospy.init_node("recored")
    record_file()
    while not rospy.is_shutdown():
        rospy.spin()


        









