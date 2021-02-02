from __future__ import print_function
import torch
import numpy as np
import os


def mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def print_network(net):
    print("============ Network initialized ===========")
    num_params = 0
    for param in net.parameters():
        num_params += param.numel()
    print('[Network] Total numpber of parameters : %.3f M' % (num_params / 1e6))
    print("============================================")
