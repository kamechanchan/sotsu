#!/usr/bin/env bash

echo 'Train with 3DCNN'
echo 'Denso Parts is '$@'!'
echo ''

for i in $@
do

## run the training
python2 train.py \
--dataroot ../datasets/ \
--dataset_model $i \
--resolution 50 \
--phase train \
--name 3DCNN \
--dataset_mode pose_estimation \
--arch 3DCNN \
--batch_size 32 \
--max_dataset_size 500 \
--num_epoch 100 \
--print_freq 10 \
--save_epoch_freq 10 \
--gpu_ids 0 \
--gpu_nums 1 \
--num_threads 8 \
--serial_batches False \
--verbose_plot True\
--lr 0.001 \

done
