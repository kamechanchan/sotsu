#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import h5py, sys, random
import numpy as np
from tqdm import tqdm
from base_loader import Base_Loader

class PCD_Loader(Base_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size):
        super(PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size)

    def load_hdf5(self):
        path = self.find_h5py_filenames(self.dir)[0]
        dir_path = self.dir+"/"+path
        self.hdf5_file = h5py.File(dir_path, "r")

        print("Start loading datasets !!")
        for n in tqdm(range(0, self.dataset_size)):
            pcl_data = self.hdf5_file["data_" + str(n + 1)]['pcl'][()]
            pose_data = self.hdf5_file["data_" + str(n + 1)]['pose'][()]
            pose_data = self.conv_quat2mat(pose_data)
            # pcl_data = random.sample(pcl_data, 1024)
            self.x_data.append(pcl_data)
            self.y_data.append(pose_data)

    def get_data(self, index):
        pcd_data = self.x_data[index]
        x_data = random.sample(pcd_data, 1024)
        return x_data, self.y_data[index]

if __name__ == "__main__":
    loader = PCD_Loader("../../datasets/", "HV8", 200)
    loader.load_hdf5()

