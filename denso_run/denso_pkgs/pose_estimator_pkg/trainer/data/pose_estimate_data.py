#!/usr/bin/env python2
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

class PoseData(BaseDataset):
    def __init__(self, opt):
        BaseDataset.__init__(self, opt)
        self.arch = opt.arch
        self.root = opt.dataroot
        self.dataset_model = opt.dataset_model
        self.dir = os.path.join(opt.dataroot, opt.name)
        self.size = opt.max_dataset_size
        self.hdf5_data = None

        if self.arch == "3DCNN":
            from voxel_loader import Voxel_Loader
            self.hdf5_data = Voxel_Loader(self.dir, self.dataset_model, self.size)
            self.hdf5_data.init_param(channel=1, div=50)
            self.hdf5_data.load_hdf5()

        elif self.arch == "PointNet_Pose":
            from pcd_loader import PCD_Loader
            self.hdf5_data = PCD_Loader(self.dir, self.dataset_model, self.size)
            self.hdf5_data.load_hdf5()
        else:
            print("pose_estimate_data.py: Error!! Could not detect hdf5 format file or Could not resolve arch type")
            sys.exit(1)

    def __getitem__(self, index):
        meta = {}
        try:
            x_data, y_data = self.hdf5_data.get_data(index)
            meta["x_data"] = x_data
            meta["y_data"] = y_data
            return meta
        except:
            print("pose_estimate_data.py: Error! Cloud not load hdf5_data")
            sys.exit(1)


    def __len__(self):
        return self.size

if __name__ == "__main__":
    opt = TrainOptions().parse()
    loader = PoseData(opt)
