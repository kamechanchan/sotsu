#!/usr/bin/env bash

## run the training
python3 train_devel.py \
--dataroot ../datasets \
--phase train \
--name 3DCNN \
--dataset_mode pose_estimation \
--arch C3D_Voxel \
--batch_size 1 \
--max_dataset_size 2 \
--num_epoch 3 \
--print_freq 10 \
--save_epoch_freq 10 \
--gpu_ids 0 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.001 \
