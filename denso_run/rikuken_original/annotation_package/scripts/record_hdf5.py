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



class record_file(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("annotation_package")
        self.filepath = rospy.get_param("~filepath", self.package_path + "/dataset")
        self.num_dataset = rospy.get_param("~num_dataset", 5)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        cloud_sub = rospy.Subscriber("all_cloud", PointCloud2, self.callback)
        rospy.set_param("/is_move/ok", True)
        self.init_file()
        self.num_ = 1
        self.matu = 0

    def init_file(self):
        util_rikuken.mkdir(self.filepath)
        self.all_file_path = self.filepath + "/init.hdf5"
        self.hdf5_file = h5py.File(self.all_file_path, "w")

    def callback(self, cloud):
        record_ok = rospy.get_param("/is_record/ok", False)
        ls = PointCloud2()
        
 
        
        if record_ok:
            self.matu += 1
        if self.matu >= 3:
            rospy.set_param("/is_record_kekkyoku/ok", True)
            pc = ros_numpy.numpify(cloud)
            #pcd_ls = pcl.PointCloud(np.array(pc, np.float32))
            #pcl.save(pcd_ls, "/home/ericlab/tameshi_pcd/init" + str(self.num_) + ".hdf5")
            '''
            height = pc.shape[0]
            width = 1
            np_points = np.zeros((height*width, 4), dtype=np.float32)
            np_points[:, 0] = np.resize(pc['x'], height*width)
            np_points[:, 1] = np.resize(pc['y'], height*width)
            np_points[:, 2] = np.resize(pc['z'], height*width)
            np_points[:, 3] = np.resize(pc['rgb'], height*width)
            #pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
            '''
            self.savePCD(pc)
            rospy.set_param("/is_move/ok", True)
            rospy.set_param("/is_record/ok", False)
            self.matu = 0
        
    def savePCD(self, cloud):
        if self.num_ >  self.num_dataset:
            rospy.signal_shutdown('finish')
            os._exit(10)
        else:
            
            data_g = self.hdf5_file.create_group("data_" + str(self.num_))
            data_g.create_dataset("pcl", data=cloud, compression="lzf")
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


        









