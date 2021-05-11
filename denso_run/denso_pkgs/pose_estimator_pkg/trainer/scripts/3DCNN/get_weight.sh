#!/usr/bin/env bash

DATADIR='weights' #location where data gets downloaded to
DATAROOT=$(cd $(dirname $0)/../../..;pwd)
DATAARCH="3DCNN"


# get data
cd $DATAROOT\/$DATADIR
mkdir -p $DATAARCH && cd $DATAARCH
wget https://www.dropbox.com/s/ndlxapo02wz9b04/latest_net.tar.gz
tar -xzvf latest_net.tar.gz && rm latest_net.tar.gz
echo "downloaded the data and putting it in: " $DATAROOT\/$DATADIR\/$DATAARCH

