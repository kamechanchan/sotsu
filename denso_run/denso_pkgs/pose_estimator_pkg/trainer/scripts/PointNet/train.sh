#!/usr/bin/env bash

python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
<<<<<<< HEAD
--dataroot /home/ericlab/OneDrive/DENSO/raugh_recognition/datasets/ \
--dataset_model HV6_size_20000_range_pi_1.hdf5 \
--dataset_mode pose_estimation \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/pose_estimate/checkpoint \
--resolution 1024 \
--phase train \
--process_swich raugh_recognition \
--batch_size 8 \
--num_epoch 200 \
--max_dataset_size 20000 \
=======
--dataroot /home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset \
--dataset_model kandou_6000.hdf5 \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/ \
--resolution 1024 \
--phase train \
--process_swich object_segment \
--dataset_mode instance_segmentation \
--batch_size 2 \
--num_epoch 200 \
--max_dataset_size 6000 \
>>>>>>> 085e21b6061a7fe9c0f41abe02d571c6a294b3eb
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
<<<<<<< HEAD
--is_train train \
--dataroot_swich tsuchida \
--checkpoints_human_swich tsuchida \
--local_checkpoints_dir /home/ericlab/DENSO_results/pose_estimate/checkpoint \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch tsuchida/0725_raugh \
--instance_number 8
=======
--checkpoints_human_swich ishiyama \
--checkpoints_process_swich object_segment \
--dataroot_swich tsuchida \
--local_checkpoints_dir /home/ericlab/DENSO_results/ \
--tensorboardX_results_directory /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/ \
--tensorboardX_results_directory_switch ishiyama/0726 \
>>>>>>> 085e21b6061a7fe9c0f41abe02d571c6a294b3eb
