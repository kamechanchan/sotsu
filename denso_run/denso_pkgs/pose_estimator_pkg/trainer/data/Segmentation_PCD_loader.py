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

class Segmentation_PCD_Loader(PCD_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number, instance_number):
        super(Segmentation_PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number)
        self.instance_number = instance_number

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

    def get_pcd_data(self, index):
        pcd_data = self.x_data[index]
        print("pcd_data")
        print(pcd_data)
        original_data, pcd_offset = getNormalizedPcd_seg(pcd_data, 8192)
        x_data = original_data[:,:3]
<<<<<<< HEAD
        print("x_data")
        print(x_data)
        pre_mask_data = original_data[:,3]
        pre_mask_data = pre_mask_data.astype(np.int64)
        y_data = np.zeros((original_data.shape[0], self.instance_number), dtype=np.float32)
        #print(type(pre_mask_data[1]))
        #print(original_data.shape[0])
        #print(y_data.shape)
        y_data[np.arange(original_data.shape[0]), pre_mask_data[:]] = 1
        # print("*******************y_data***************************")
        # print(y_data)
        # cnt = 0
        # for i in range(4096) :
        #     if y_data[i,7] == 1:
        #         cnt+=1
        # print("******************cnt********************")        
        # print(cnt)
        y_data = np.delete(y_data,self.instance_number-1,1)
        # print("y_data")
        # print(y_data.shape[1])
        # y_data_size = y_data.shape[1]
        sizes = 0
        cnt_instance = [0]*y_data.shape[1]
        instance_choice = []
        for j in range(y_data.shape[1]):
            for i in range(8192):
                if y_data[i,j] == 1:
                    cnt_instance[j] += 1
        for i in range(len(cnt_instance)):
            if cnt_instance[i] == 0:
                instance_choice.append(i)
        # print("afj")
        cnt = 0
        if instance_choice:
            for i in range(len(instance_choice)):
                # y_data = np.delete(y_data,instance_choice[i],1)
                # print(y_data.shape)
                # y_data = y_data.transpose(instance_choice[i],self.instance_number-2-i)
                once_save = []
                once_save = np.array(once_save)
                once_save = y_data[:,instance_choice[len(instance_choice)-i-1]]
                y_data[:,instance_choice[len(instance_choice)-i-1]] = y_data[:,self.instance_number-2-i]
                y_data[:,self.instance_number-2-i] = once_save
                # print(i)
                cnt += 1
        sizes = y_data.shape[1] - cnt
        # print(sizes)
        # if sizes == 5:
        #     # print(y_data)
        #     cnt_a = 0
        #     cnt_b = 0
        #     cnt_c = 0
        #     cnt_d = 0
        #     cnt_e = 0
        #     cnt_f = 0
        #     cnt_g = 0
        #     # cnt_l = 0
        #     for i in range(1024) :
        #         if y_data[i,0] == 1:
        #             cnt_a+=1
        #         elif y_data[i,1] == 1:
        #             cnt_b+=1
        #         elif y_data[i,2] == 1:
        #             cnt_c+=1
        #         elif y_data[i,3] == 1:
        #             cnt_d+=1
        #         elif y_data[i,4] == 1:
        #             cnt_e+=1
        #         elif y_data[i,5] == 1:
        #             cnt_f+=1
        #         elif y_data[i,6] == 1:
        #             cnt_g+=1
        #     print("cnt")
        #     print(cnt_a)
        #     print(cnt_b)
        #     print(cnt_c)
        #     print(cnt_d)
        #     print(cnt_e)
        #     print(cnt_f)
        #     print(cnt_g)
        #     print("instance")
        #     print(instance_choice[0])
        #     print(instance_choice[1])
        # elif sizes == 4:
        #     # print(y_data)
        #     cnt_a = 0
        #     cnt_b = 0
        #     cnt_c = 0
        #     cnt_d = 0
        #     cnt_e = 0
        #     cnt_f = 0
        #     cnt_g = 0
        #     # cnt_l = 0
        #     for i in range(1024) :
        #         if y_data[i,0] == 1:
        #             cnt_a+=1
        #         elif y_data[i,1] == 1:
        #             cnt_b+=1
        #         elif y_data[i,2] == 1:
        #             cnt_c+=1
        #         elif y_data[i,3] == 1:
        #             cnt_d+=1
        #         elif y_data[i,4] == 1:
        #             cnt_e+=1
        #         elif y_data[i,5] == 1:
        #             cnt_f+=1
        #         elif y_data[i,6] == 1:
        #             cnt_g+=1
        #     print("cnt")
        #     print(cnt_a)
        #     print(cnt_b)
        #     print(cnt_c)
        #     print(cnt_d)
        #     print(cnt_e)
        #     print(cnt_f)
        #     print(cnt_g)
        #     print("instance")
        #     print(instance_choice[0])
        #     print(instance_choice[1])
        #     print(instance_choice[2])
=======
        
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
>>>>>>> 085e21b6061a7fe9c0f41abe02d571c6a294b3eb

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

        return x_data, y_data, sizes