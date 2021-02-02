#!/usr/bin/env bash

## run the training
python2 test.py \
--dataroot ../datasets \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 3 \
--max_dataset_size 100 \
--num_epoch 500 \
--arch PointNet \
--print_freq 10 \
--save_epoch_freq 10 \
--gpu_ids 0 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.0001 \

