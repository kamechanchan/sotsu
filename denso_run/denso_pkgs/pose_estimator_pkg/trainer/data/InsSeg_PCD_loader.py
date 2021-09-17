from __future__ import print_function
from numpy.core.fromnumeric import size
from tqdm import tqdm
import h5py
from pcd_loader import PCD_Loader
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../../utils'))
import numpy as np
from cloud_util import *
import h5py
import pcl
from utils import util


class InsSeg_PCD_Loader(PCD_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number, opt):
        super(InsSeg_PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number, opt)
        self.instance_number = opt.instance_number
        self.dataset_mode = opt.dataset_mode
        # self.dataset_model = dataset_model
        self.concat_dataset_model = '+'.join(dataset_model)
        self.is_trian = opt.is_train

    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            # print("*******************start*********************")
            # print(dir_path)
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                pcl_data = self.hdf5_file["data_" + str(n + 1)]['Points'][()]
                mask_data = self.hdf5_file["data_" + str(n + 1)]['masks'][()]
                prepare_data = np.hstack([pcl_data, mask_data])
                self.x_data.append(prepare_data)

    def get_pcd_data(self, index, resolution):
        pcd_data = self.x_data[index]
        # print("pcd_data")
        # print(pcd_data)
        original_data, pcd_offset = getNormalizedPcd_seg(pcd_data, resolution)
        x_data = original_data[:,:3]
        y_data = original_data[:,3]
        # print("majikatsuchidasan")

        if self.is_trian == True:
            # print("godtsuchida")
            # print(type(x_data))
            
            pcl_visu = pcl.PointCloud(x_data)
            pcd_dir = "/home/ericlab/DENSO_results/August/pcl_visu/train_input/"+self.dataset_mode+"/"+self.concat_dataset_model
            util.mkdir(pcd_dir)
            pcl.save(pcl_visu, pcd_dir+"/result"+str(index)+".pcd")

            # print(y_data.shape)
            # print(y_data.dtype)
            # instance_segmentation
            pre_mask_data = original_data[:,3]
            pre_mask_data = pre_mask_data.astype(np.int64)
            y_data = np.zeros((original_data.shape[0], self.instance_number), dtype=np.float32)
            #print(type(pre_mask_data[1]))
            #print(original_data.shape[0])
            #print(y_data.shape)
            y_data[np.arange(original_data.shape[0]), pre_mask_data[:]] = 1
            # print(y_data.shape[1])
            # for i in range(1000):
            #     for j in range(30):
            #         print(i)
            # cnt_list = [0] * y_data.shape[1]
            # for i in range(resolution):
            #     for j in range(y_data.shape[1]):
            #         if original_data[i,3] == j:
            #             cnt_list[j] += 1
            # print(y_data.shape)

            # cnt_a = 0
            # cnt_b = 0
            # cnt_c = 0
            # cnt_d = 0
            # cnt_e = 0
            # cnt_f = 0
            # cnt_g = 0
            # # cnt_l = 0
            # for i in range(100000) :
            #     if y_data[i,0] == 1:
            #         cnt_a+=1
            #     elif y_data[i,1] == 1:
            #         cnt_b+=1
            #     elif y_data[i,2] == 1:
            #         cnt_c+=1
            #     elif y_data[i,3] == 1:
            #         cnt_d+=1
            #     elif y_data[i,4] == 1:
            #         cnt_e+=1
            #     elif y_data[i,5] == 1:
            #         cnt_f+=1
            #     elif y_data[i,6] == 1:
            #         cnt_g+=1
            #     # elif y_data[i,7] == 1:
            #     #     cnt_l+=1
            # print("******************cnt********************")        
            # print("cnt_a")
            # print(cnt_a)
            # print("cnt_b")
            # print(cnt_b)
            # print("cnt_c")
            # print(cnt_c)
            # print("cnt_d")
            # print(cnt_d)
            # print("cnt_e")
            # print(cnt_e)
            # print("cnt_f")
            # print(cnt_f)
            # print("cnt_g")
            # print(cnt_g)
            # print("cnt_l")
            # print(cnt_l)
            
            # print("*******************aranege_y_data***************************")
            # print(y_data)

            # return x_data, y_data, sizes
        return x_data, y_data