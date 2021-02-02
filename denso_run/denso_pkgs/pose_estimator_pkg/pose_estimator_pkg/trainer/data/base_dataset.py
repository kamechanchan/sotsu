#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch.utils.data as data
import numpy as np
import pickle
import os

class BaseDataset(data.Dataset):
    def __init__(self, opt):
        self.opt = opt
        self.mean = 0
        self.std = 1
        self.ninput_channels = None
        super(BaseDataset, self).__init__()


def collate_fn(batch):
    meta = {}
    keys = batch[0].keys()
    for key in keys:
        meta.update({key:np.array([d[key] for d in batch])})
    return meta
