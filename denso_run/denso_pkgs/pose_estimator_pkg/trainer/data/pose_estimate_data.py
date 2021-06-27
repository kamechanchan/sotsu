#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import torch
import os, csv, random, sys
import numpy as np
from tqdm import tqdm

from options.train_options import TrainOptions
from data.base_dataset import BaseDataset
from pcd_loader import PCD_Loader
from voxel_loader import Voxel_Loader
from Segmentation_PCD_loader import Segmentation_PCD_Loader


class PoseData(BaseDataset):
    def __init__(self, opt):
        BaseDataset.__init__(self, opt)
        self.arch = opt.arch
        #self.root = opt.dataroot
        self.dataset_model = opt.dataset_model
        self.dir = opt.dataroot
        self.dataroot_swich=opt.dataroot_swich
        self.dataroot=os.path.join(self.dir,self.dataroot_swich)
        self.resolution = opt.resolution
        self.size = opt.max_dataset_size
        self.len_size = 0
        self.dataset_number = opt.dataset_number
        self.hdf5_data = None

        self.hdf5_data = PCD_Loader(self.dataroot,self.dataset_model, self.size, self.dataset_number)
        # self.hdf5_data = Voxel_Loader(self.dir, self.dataset_model, self.size)
        # self.hdf5_data.init_param(1, self.resolution)
        self.hdf5_data.load_hdf5()

        for i in range(self.dataset_number):
            self.len_size = self.len_size + self.size[i]

    def __getitem__(self, index):
        meta = {}
        try:
            if self.arch == "PointNet_Pose":
                x_data, y_data = self.hdf5_data.get_pcd_data(index)
                meta["x_data"] = x_data
                meta["y_data"] = y_data
                return meta
            elif self.arch == "3DCNN":
                x_data, y_data = self.hdf5_data.get_voxel_data(index, self.resolution)
                meta["x_data"] = x_data
                meta["y_data"] = y_data
                return meta
        except:
            print("pose_estimate_data.py: Error! Cloud not load hdf5_data")
            sys.exit(1)


    def __len__(self):
        return self.len_size


class Segmentation_Data(PoseData):
    def __init__(self, opt):
        PoseData.__init__(self, opt)
        self.instance_number = opt.instance_number
        self.hdf5_data = Segmentation_PCD_Loader(self.dataroot,self.dataset_model, self.size, self.dataset_number, self.instance_number)
        self.hdf5_data.load_hdf5()

    def __getitem__(self, index):
        meta = {}
        try:
            if self.arch == "Segmentation_PointNet":
                x_data, y_data = self.hdf5_data.get_pcd_data(index)
                meta["x_data"] = x_data
                meta["y_data"] = y_data
                #meta["sizes"] = sizes
                return meta
        except:
            print("pose_estimate_data.py: Error! Cloud not load hdf5_data")
            sys.exit(1)


if __name__ == "__main__":
    opt = TrainOptions().parse()
    loader = PoseData(opt)
