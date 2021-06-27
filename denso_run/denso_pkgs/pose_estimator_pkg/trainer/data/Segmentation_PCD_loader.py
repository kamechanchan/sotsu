from tqdm import tqdm
import h5py
from pcd_loader import PCD_Loader
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
import numpy as np
from cloud_util import *
import h5py

class Segmentation_PCD_Loader(PCD_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number, instance_number):
        super(Segmentation_PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number)

    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                #pcl_data = self.hdf5_file["data_" + str(n + 1)]['points'][()]
                #mask_data = self.hdf5_file["data_" + str(n + 1)]['masks'][()]
                pcl_data = self.hdf5_file["data_"]['Points'][()]
                mask_data = self.hdf5_file["data_"]['masks'][()]
                print("aaaaaaaaaaaaa")
                print(mask_data)

                self.x_data.append(pcl_data)
                self.y_data.append(mask_data)

    def get_pcd_data(self, index):
        pcd_data = self.x_data[index]
        x_data, pcd_offset = getNormalizedPcd(pcd_data, 4096)
        y_data = self.y_data[index]
        y_pos = y_data[0:3] - pcd_offset
        y_rot = y_data[3:]
        y_data = np.concatenate([y_pos, y_rot])

        return x_data, y_data