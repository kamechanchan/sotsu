import numpy as np
import h5py


def PrintonlyDataset(name, obj):
    # print(name)
    if isinstance(obj, h5py.Dataset):
        print(name)

def PrintAllObjects(name):
    print(name)

cnt = 0
c = 1
with h5py.File('/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset/instance_tsuchida_10000_8_7_5_12.hdf5', mode="r") as f:
    # for i in f["data_1"].values():
    #     # cnt +=1
    #     # c += 1
    #     # if cnt == 5000:
    #     print(i)
    #         # cnt =0
    # f.visititems(PrintonlyDataset)
    f.visit(PrintAllObjects)

