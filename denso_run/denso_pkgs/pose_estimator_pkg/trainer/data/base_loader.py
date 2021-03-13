#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import h5py, sys, random
from tqdm import tqdm
from tf.transformations import euler_matrix, quaternion_matrix
from os import listdir
import numpy as np

class Base_Loader(object):
    def __init__(self, dir_name, dataset_model, dataset_size):
        self.dir = dir_name
        self.dataset_model = dataset_model
        self.dataset_size = dataset_size
        self.x_data = []
        self.y_data = []
        self.dataset = []
        self.h5_file = None

    def conv_euler2mat(self, vec):
        conv_vec = np.zeros(12)
        conv_vec[0:3] = vec[0:3]
        trans_euler =euler_matrix(vec[3], vec[4], vec[5], "sxyz")
        conv_vec[3:12] = trans_euler[0:3, 0:3].reshape(9)
        return conv_vec

    def conv_quat2mat(self, vec):
        conv_vec = np.zeros(12)
        conv_vec[0:3] = vec[0:3]
        trans_euler =quaternion_matrix(vec[3:7])
        conv_vec[3:12] = trans_euler[0:3, 0:3].reshape(9)
        return conv_vec


    def get_data(self, index):
        return self.x_data[index], self.y_data[index]

    def find_h5py_filenames(self, path_to_dir, suffix=".hdf5"):
        filenames = listdir(path_to_dir)
        file_list = [filename for filename in filenames if filename.endswith(suffix)]
        file_name = [f for f in file_list if self.dataset_model in f]
        if len(file_name) == 1:
            return file_name
        else:
            print(str(path_to_dir))
            print("Error, Cloud not load h5py data file!! or detect multi hdf5 file !!")
            sys.exit(1)
