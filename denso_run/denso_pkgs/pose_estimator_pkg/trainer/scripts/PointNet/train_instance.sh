#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/hdf5_data/ \
--dataset_model instance_changed_8_7_1526.hdf5 instance_changed_instance_tsuchida_8_12_500_1.hdf5 instance_changedinstance_tsuchida_8_11_1000_1.hdf5 \
--dataset_mode instance_segmentation \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/August \
--resolution 8192 \
--phase train \
--process_swich object_segment \
--batch_size 2 \
--num_epoch 100 \
--max_dataset_size 1526 500 1000\
--arch JSIS3D \
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
--dataroot_swich instance \
--checkpoints_human_swich ishiyama \
--local_checkpoints_dir /home/ericlab/DENSO_results/August \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch tsuchida/0817_original \
--instance_number 25