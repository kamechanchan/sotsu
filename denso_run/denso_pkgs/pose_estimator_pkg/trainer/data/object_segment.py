#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys

from traitlets.traitlets import Instance
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

class Segmentation_Data(BaseDataset):
    def __init__(self, opt):
        BaseDataset.__init__(self, opt)
        self.instance_number = opt.instance_number
        self.hdf5_data = Segmentation_PCD_Loader(self.dataroot, self.dataset_model, self.size, self.dataset_number, self.instance_number)
        self.hdf5_data.load_hdf5()

    def __getitem__(self, index):
        # meta_x = []
        # meta_y = []
        meta = {}
        #try:
<<<<<<< HEAD
        if self.arch == "JSIS3D":
            x_data, y_data, sizes = self.hdf5_data.get_pcd_data(index)
            # meta_x.append(x_data)
            # meta_y.append(y_data)
            meta["x_data"] = x_data
            meta["y_data"] = y_data
            meta["sizes"] = sizes
            return meta
=======
        # if self.arch == "JSIS3D":
        x_data, y_data = self.hdf5_data.get_pcd_data(index)
        # print("x_data")
        # print(x_data)
        # print("y_data")
        # print(y_data)
        meta["x_data"] = x_data
        meta["y_data"] = y_data
        #meta["sizes"] = sizes
        return meta
>>>>>>> 085e21b6061a7fe9c0f41abe02d571c6a294b3eb
        """
        except:
            print("pose_estimate_data(Segmentation_Data).py: Error! Cloud not load hdf5_data")
            sys.exit(1)
        """

    def __len__(self):
        return self.len_size