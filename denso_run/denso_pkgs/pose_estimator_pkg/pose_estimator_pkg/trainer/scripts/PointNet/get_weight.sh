#!/usr/bin/env bash

DATADIR='weights' #location where data gets downloaded to
DATAROOT=$(cd $(dirname $0)/../../..;pwd)
DATAARCH="PointNet"


# get data
cd $DATAROOT\/$DATADIR
mkdir -p $DATAARCH && cd $DATAARCH
wget https://www.dropbox.com/s/ne9i8de4w1qq9t1/latest_net.tar.gz
tar -xzvf latest_net.tar.gz && rm latest_net.tar.gz
echo "downloaded the data and putting it in: " $DATAROOT\/$DATADIR\/$DATAARCH

