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
        self.instance_number = instance_number

    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                pcl_data = self.hdf5_file["data_" + str(n + 1)]['Points'][()]
                mask_data = self.hdf5_file["data_" + str(n + 1)]['masks'][()]
                prepare_data = np.hstack([pcl_data, mask_data])
                self.x_data.append(prepare_data)

    def get_pcd_data(self, index):
        pcd_data = self.x_data[index]
        original_data, pcd_offset = getNormalizedPcd_seg(pcd_data, 4096)
        x_data = original_data[:,:3]
        
        y_data = original_data[:,3]
        # print(y_data.dtype)
        # instance_segmentation
        # pre_mask_data = original_data[:,3]
        # pre_mask_data = pre_mask_data.astype(np.int64)
        # y_data = np.zeros((original_data.shape[0], self.instance_number), dtype=np.float32)
        # #print(type(pre_mask_data[1]))
        # #print(original_data.shape[0])
        # #print(y_data.shape)
        # y_data[np.arange(original_data.shape[0]), pre_mask_data[:]] = 1

        return x_data, y_data