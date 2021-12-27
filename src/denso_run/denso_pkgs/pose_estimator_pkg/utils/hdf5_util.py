import h5py
from tqdm import tqdm
import os
import sys
import re

import open3d as o3d
import numpy as np
import random
import dask.dataframe as ddf


args = sys.argv
current_path = os.getcwd()

def make_dataset_path(path):
    path2datas = []
    assert os.path.isdir(path), '%s is not a valid directory' % path

    for root, _, fnames in sorted(os.walk(path)):
        for fname in fnames:
            path = os.path.join(root, fname)
            path2datas.append(path)

        name = lambda val : int(re.sub("\\D", "", val))
        path2datas = sorted(path2datas, key=name)
        return path2datas

def write_out(fname, key, data, d_type):
    dataset_size = len(data)

    output_file_name = fname +"_"  + key +  "_" + str(dataset_size) + ".hdf5"

    hfile = h5py.File(output_file_name, "w")
    for i, d in enumerate(data):
        d_shape = d.shape
        print(d_shape)
        hfile.create_dataset(key + str(i+1), data=d, shape=d_shape, dtype=d_type)
    hfile.close()


def read_csv2np(path):
    df = ddf.read_csv(path, delim_whitespace=True, header=None)
    df = df.compute()
    df = df.values.T
    return df


target_path = []

for i, arg in enumerate(args[1:]):
    path = os.path.join(current_path, arg)
    target_path.append(path)


for path in target_path:
    fname = make_dataset_path(path)

    if path.endswith("pcd"):
        pcd_data = []
        for f in tqdm(fname[:10]):
            if f.endswith("pcd") == True:
                pcd = o3d.io.read_point_cloud(f)
                xyz = np.asarray(pcd.points)
                #xyz = random.sample(xyz, 1000)
                pcd_data.append(xyz)
            else:
                print("Contains unorgaized format in the dir")
                break

        write_out("hdf5_out", "pcd", pcd_data, "float32")

    elif path.endswith("pose"):
        pose_data = []
        for f in tqdm(fname[:10]):
            if f.endswith("csv") == True:
                csv_data = read_csv2np(f)
                pose_data.append(csv_data)
            else:
                print("Contains unorgaized format in the dir")
                break
        write_out("hdf5_out", "pose", pose_data, "float32")

