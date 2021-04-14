#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import h5py, sys, random
import numpy as np
from tqdm import tqdm
from base_loader import Base_Loader

class Voxel_Loader(Base_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size):
        super(Voxel_Loader, self).__init__(dir_name, dataset_model, dataset_size)
        self.channel = None
        self.div = None

    def init_param(self, channel=1, div=50):
        self.channel=channel
        self.div = div
        self.x_data = np.zeros((self.dataset_size, self.channel, self.div, self.div, self.div), dtype="float32")
        self.y_data = np.zeros((self.dataset_size, 12), dtype="float32")

    def load_hdf5(self):
        path = self.find_h5py_filenames(self.dir)[0]
        dir_path = self.dir + "/" + path
        self.hdf5_file = h5py.File(dir_path, "r")

        print("Start loading datasets !!")
        for n in tqdm(range(1, self.dataset_size)):
            voxel = self.hdf5_file["data_" + str(n)]['voxel'][()]
            pose = self.hdf5_file["data_" + str(n)]['pose'][()]
            self.x_data[n, 1 - 1] = voxel.reshape(self.div, self.div, self.div)
            self.y_data[n] = self.conv_euler2mat(pose)


if __name__ == "__main__":
    loader = Voxel_Loader("../../datasets/3DCNN/", 10000)
    loader.init_param(channel=1, div=50)
    loader.load_hdf5()

