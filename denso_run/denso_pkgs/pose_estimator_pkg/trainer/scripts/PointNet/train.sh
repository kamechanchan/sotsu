#!/usr/bin/env bash

<<<<<<< HEAD
## run the training
python3 /home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/tsuchidashinya/MEGAsync/Precision-7820-Tower \
--dataset_model  dateset_20000_1.hdf5\
--checkpoints_dir /home/tsuchidashinya/MEGAsync/X10/03_19 \
=======
python3 /home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/ericlab/OneDrive/DENSO/raugh_recognition/datasets \
--dataset_model dataset_20000.hdf5 \
--checkpoints_dir /home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint \
>>>>>>> a3449ccc3e265eb954266948d4879cbd6a1ec396
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
<<<<<<< HEAD
--batch_size 32 \
--max_dataset_size 20000 \
--num_epoch 800 \
=======
--batch_size 8 \
--num_epoch 800 \
--max_dataset_size 20000 \
>>>>>>> a3449ccc3e265eb954266948d4879cbd6a1ec396
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 1000 \
--save_epoch_freq 1 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
<<<<<<< HEAD
--num_threads 8 \
=======
--num_threads 0 \
>>>>>>> a3449ccc3e265eb954266948d4879cbd6a1ec396
--serial_batches False \
--verbose_plot True \
--lr 0.0001 \
--checkpoints_swich ishiyama/0406 \
--dataroot_swich front \
--local_checkpoints_dir /home/ericlab/DENSO_results/raugh_recognition/checkpoint \
