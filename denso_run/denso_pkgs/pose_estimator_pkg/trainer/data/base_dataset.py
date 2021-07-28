#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sys import pycache_prefix
import torch
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
    keys = batch[0].keys()
    # print("dadata")
    # print(keys)
    # for d in batch:
    #     print("dat")
    #     print(d)
    for key in keys:
        # for d in batch:
        #     print("da")
        #     print(d[key].shape)
        meta.update({key:np.array([d[key] for d in batch])})

    return meta

def collate_fn_original(batch):
    meta_x = []
    meta_y = []
    # meta_y = np.array(meta_y)

    for sample in batch:
        # print("x_data")
        # print(np.shape(sample[1]))
        meta_x.append(sample[0])
        meta_y.append(torch.FloatTensor(sample[1]))
        # meta_y = np.append(meta_y, sample[1])
        # print(meta_x.type)
    # for i in batch:
    #     meta_y[i] = np.array(meta_y[i])
    # print("meta")
    # print(type(meta_y))
    # print(np.shape(meta_x))
    # print(np.shape(meta_y))
    meta_x = np.array(meta_x)
    # meta_y = np.array(meta_y)
    meta_x = torch.from_numpy(meta_x.astype(np.float32))
    # meta_y = torch.from_numpy(meta_y.astype(np.float32))
    # print(type(meta_y))
    # print(meta_x.shape)
    # print(type(meta_y[1]))
    # meta_x = torch.stack(meta_x)
    # meta_y = torch.stack(meta_y)
    # print(meta_y)
    # print(np.shape(meta_y[1])[1])

    return meta_x, meta_y
