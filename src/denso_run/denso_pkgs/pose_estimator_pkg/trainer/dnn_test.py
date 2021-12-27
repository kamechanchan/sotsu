#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys, os
from typing import IO

from numpy.core.numeric import indices

from utils.util import print_network

sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from options.test_options import TestOptions
from options.train_options import TrainOptions
from data import *
from models import create_model
import time
from color_cloud_bridge.msg import out_segmentation
import numpy as np
from sklearn.cluster import KMeans


def run_test(opt, dataset):

    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input(data)
        loss = model.val_step()
        time_end = time.time()

        val_loss += loss
    return val_loss

def run_progress_savetest(opt, dataset, epoch):
    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input(data)
        model.progress_save_pcd(opt, epoch, i)
        time_end = time.time()

    return

def run_segmentation_test(opt, dataset):

    opt.serial_batches = True
    val_loss = 0.0
    model = create_model(opt)

    for i, data in enumerate(dataset):
        time_sta = time.time()

        model.set_input_segmentation(data)
        loss = model.val_step()
        time_end = time.time()

        val_loss += loss
    return val_loss


def estimation(model, data):

    time_sta = time.time()
    print("estimation")
    print(model)
    print(data.shape)
    model.set_input(data)
    pred = model.test_step()
    time_end = time.time()

    return pred, (time_end - time_sta)

def estimation_acc(model, data, resolution, dataset_mode, instance_number):

    x_data = data["x_data"]
    y_data = data["y_data"]
    num_class = 2
    accu  = np.zeros(num_class)
    freq  = np.zeros(num_class)
    inter = np.zeros(num_class)
    union = np.zeros(num_class)
    # IoU_com = [] * 4
    # IoU_com = np.array(IoU_com)
    # print("y_data")
    # print(y_data.shape)
    msg_out = out_segmentation()

    model.set_input_acc(data)
    time_sta = time.time()
    pred = model.acc_step()

    # print("pred_pred::::")
    # print(pred.shape)

    pred_new = []
    if dataset_mode == "instance_segmentation":
        for i in range(pred.shape[0]):
            pred_new.append(KMeans(n_clusters=instance_number).fit_predict(pred[i,:,:]))
        pred_new = np.array(pred_new)
    elif dataset_mode == "semantic_segmentation":
        pred_new = pred
    else:
        print("error")
        sys.exit(1)
    # print("************pred_new***********")
    # print(np.array(pred_new).shape)
    print("tamo")
    print(x_data.shape)
    time_end = time.time()
    for i in range(x_data.shape[0]):
        for j in range(resolution):
            msg_out.x.append(x_data[i][j][0])
            msg_out.y.append(x_data[i][j][1])
            msg_out.z.append(-x_data[i][j][2])
            msg_out.instance.append(pred_new[i][j])

    return msg_out, (time_end - time_sta), pred_new


if __name__ == "__main__":

    opt = TrainOptions().parse()
    opt_v = TestOptions().parse()
    train_dataset, val_dataset = TrainValDataset(opt)
    val_dataset = ValDataLoader(val_dataset, opt_v)
    run_test(opt, val_dataset)
