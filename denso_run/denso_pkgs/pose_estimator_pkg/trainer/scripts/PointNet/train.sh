#!/usr/bin/env bash

echo 'Train with PointNet'
<<<<<<< HEAD
echo 'Dataset Model is '$@'/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/datasets/photoneo_center_optical_frame_HV8.hdf5'
=======
echo 'Dataset Model is  ''/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/datasets/photoneo_center_optical_frame_HV8.hdf5'
>>>>>>> b8a4e201324bbf227462141351058788a24b490f
echo ''

for i in '/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/datasets/photoneo_center_optical_frame_HV8.hdf5'
do

## run the training
python3 /home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/train.py \
--dataroot /home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/datasets \
--dataset_model  photoneo_center_optical_frame_HV8.hdf5\
--resolution 1024 \
--phase train \
--name PointNet \
--dataset_mode pose_estimation \
--batch_size 32 \
--max_dataset_size 500 \
--num_epoch 600 \
--arch PointNet_Pose \
--print_freq 10 \
--save_latest_freq 1000 \
--save_epoch_freq 10 \
--run_test_freq 1 \
--gpu_ids 0 \
--gpu_num 3 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.0001 \

done
