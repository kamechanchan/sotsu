#!/usr/bin/env bash

export CUDA_VISIBLE_DEVICES=0
# bash scripts/PointNet/train.sh HV6 HV7 HV8
# bash scripts/PointNet/train.sh HV6
bash scripts/3DCNN/train.sh HV7
