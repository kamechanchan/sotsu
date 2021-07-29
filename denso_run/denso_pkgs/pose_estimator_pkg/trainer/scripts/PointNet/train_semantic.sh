#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset \
--dataset_model semantic_6000.hdf5 \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/ \
--resolution 1024 \
--phase train \
--process_swich object_segment \
--dataset_mode semantic_segmentation \
--batch_size 2 \
--num_epoch 200 \
--max_dataset_size 6000 \
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
--checkpoints_process_swich object_segment \
--dataroot_swich tsuchida \
--local_checkpoints_dir /home/ericlab/DENSO_results/ \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch ishiyama/0727 \
--instance_number 2 \
