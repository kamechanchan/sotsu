#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/hdf5_data/ \
--dataset_model semantic_changed_8_7_1526.hdf5 \
--dataset_mode semantic_segmentation \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/August \
--resolution 8192 \
--phase train \
--process_swich object_segment \
--batch_size 2 \
--num_epoch 200 \
--max_dataset_size 1526 \
--arch PointNet_Segmentation \
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
--is_train True \
--checkpoints_human_swich ishiyama \
--dataroot_swich semantic \
--local_checkpoints_dir /home/ericlab/DENSO_results/August \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch ishiyama/0817 \
--instance_number 2 \
