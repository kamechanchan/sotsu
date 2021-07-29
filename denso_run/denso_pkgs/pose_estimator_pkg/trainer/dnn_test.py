#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
from options.test_options import TestOptions
from options.train_options import TrainOptions
from data import *
from models import create_model
import time

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
    model.set_input(data)
    pred = model.test_step()
    time_end = time.time()

    return pred, (time_end - time_sta)


if __name__ == "__main__":

    opt = TrainOptions().parse()
    opt_v = TestOptions().parse()
    train_dataset, val_dataset = TrainValDataset(opt)
    val_dataset = ValDataLoader(val_dataset, opt_v)
    run_test(opt, val_dataset)
