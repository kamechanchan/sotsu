#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/OneDrive/DENSO/raugh_recognition/datasets \
--dataset_model HV6_size_20000_range_pi_1.hdf5 HV7_size_20000_range_pi_1.hdf5 HV8_size_80000_range_pi_1.hdf5 t_pipe_size_20000_range_pi_1.hdf5 \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint \
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 8 \
--num_epoch 100 \
--max_dataset_size 20000 20000 20000 20000 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 20000 \
--save_epoch_freq 5 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
--num_threads 0 \
--serial_batches False \
--verbose_plot True \
--lr 0.0001 \
--checkpoints_swich ishiyama/0518 \
--dataroot_swich tsuchida \
--local_checkpoints_dir /home/ericlab/DENSO_results/raugh_recognition/checkpoint \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch ishiyama/0518 \