#!/usr/bin/env bash

## run the training
python3 /home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/tsuchidashinya/MEGAsync/Precision-7820-Tower \
--dataset_model  dateset_20000_1.hdf5\
--checkpoints_dir /home/tsuchidashinya/MEGAsync/X10/03_19 \
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 32 \
--max_dataset_size 20000 \
--num_epoch 800 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 1000 \
--save_epoch_freq 10 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 1 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.0001 \

done
