#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch.utils.data as data
import numpy as np
import pickle
import os

class BaseDataset(data.Dataset):
    def __init__(self, opt):
        super(BaseDataset, self).__init__()
        self.opt = opt
        self.mean = 0
        self.std = 1
        self.ninput_channels = None
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
        
        for i in range(self.dataset_number):
            self.len_size = self.len_size + self.size[i]


def collate_fn(batch):
    meta = {}
    #print(np.array(batch))
    # print("JJHHJHLKJKH")
    # print(batch)
    keys = batch[0].keys()
    # print(keys)
    for key in keys:
        meta.update({key:np.array([d[key] for d in batch])})
    return meta
