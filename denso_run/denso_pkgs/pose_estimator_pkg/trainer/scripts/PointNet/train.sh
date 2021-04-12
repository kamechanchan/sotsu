#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/OneDrive/DENSO/raugh_recognition/datasets \
--dataset_model dataset_20000.hdf5 \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint \
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 8 \
--num_epoch 800 \
--max_dataset_size 20000 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 1000 \
--save_epoch_freq 100 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
--num_threads 0 \
--serial_batches False \
--verbose_plot True \
--lr 0.0001 \
--checkpoints_swich ishiyama/0406 \
--dataroot_swich front \
--local_checkpoints_dir /home/ericlab/DENSO/raugh_recognition/checkpoint \
