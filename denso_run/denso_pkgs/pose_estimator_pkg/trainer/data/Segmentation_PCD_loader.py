import tqdm
import h5py
from pcd_loader import PCD_Loader

class Segmentation_PCD_Loader(PCD_Loader):
    def __init__(self, dir_name, dataset_model, dataset_size, dataset_number):
        super(Segmentation_PCD_Loader, self).__init__(dir_name, dataset_model, dataset_size, dataset_number)

    def load_hdf5(self):
        for i in range(self.dataset_number):
            path = self.find_h5py_filenames(self.dir)[i] #get file_name
            dir_path = self.dir+"/"+path #get path
            self.hdf5_file = h5py.File(dir_path, "r")

            print("Start loading datasets !!")
            for n in tqdm(range(0, self.dataset_size[i])):
                pcl_data = self.hdf5_file["data_" + str(n + 1)]['pcl'][()]
                mask_data = self.hdf5_file["data_" + str(n + 1)]['masks'][()]
                self.x_data.append(pcl_data)
                self.y_data.append(mask_data)

    def get_pcd_data(self, index):
        pcd_data = self.x_data[index]
        x_data, pcd_offset = getNormalizedPcd(pcd_data, 1024)
        y_data = self.y_data[index]
        y_pos = y_data[0:3] - pcd_offset
        y_rot = y_data[3:]
        y_data = np.concatenate([y_pos, y_rot])

        return x_data, y_data