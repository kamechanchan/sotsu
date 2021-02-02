#!/usr/bin/env bash

## run the training
python2 train.py \
--dataroot ../datasets \
--dataset_model HV8 \
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 32 \
--max_dataset_size 1000 \
--num_epoch 400 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 10000 \
--save_epoch_freq 1 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 3 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.0001 \

