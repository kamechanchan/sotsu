#!/usr/bin/env bash

DATADIR='datasets' #location where data gets downloaded to
DATAROOT=$(cd $(dirname $0)/../../..;pwd)
DATAARCH="3DCNN"


# get data
cd $DATAROOT\/$DATADIR
mkdir -p $DATAARCH && cd $DATAARCH
wget https://www.dropbox.com/s/e22ccr7tlhsen3i/arr_test_50000.hdf5
#tar -xzvf arr_test_50000_hdf5.tar.gz && rm arr_test_50000_hdf5.tar.gz
echo "downloaded the data and putting it in: " $DATAROOT\/$DATADIR\/$DATAARCH

