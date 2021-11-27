from types import prepare_class
import h5py
import numpy as np
from numpy.testing._private.utils import print_assert_equal


def getNormalizedPcd(np_cloud, resolution):
    """
    cloud_start=pcl.PointCloud(cloud_data)
    pre_cloud = cloud_start.make_voxel_grid_filter()
    pcl.VoxelGridFilter.set_leaf_size(pre_cloud, 0.0035, 0.0035, 0.0035)
    cloud_filter=pcl.VoxelGridFilter.filter(pre_cloud)
    np_cloud=np.array(cloud_filter)
    """ 
    pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
    pcd_data = np_cloud - pcd_offset  #original
    #pcd_data = np.asarray(np_cloud)  #improve
    choice_index = np.arange(pcd_data.shape[0])
    choice = np.random.choice(choice_index, resolution)
    normalized_pcd = pcd_data[choice, :]
    #new_pcd = pcl.PointCloud(np.array(normalized_pcd, np.float32))
    #pcl.save(new_pcd, "/home/ericlab/random_original.pcd")
    #pcl.save(new_pcd, '/home/ericlab/random_improve.pcd')
    return normalized_pcd, pcd_offset[0]

# change here by using dataset
instance_number = 26
dataset_number = 5000
resolution = 1100
# resolution_second = 10000
dataset_name_in = "HV8_size_20000_range_pi_2.hdf5"

data_root = "/home/ericlab/hdf5_data/temmat"
data_switch_in = "/"
data_path_in = data_root + data_switch_in + dataset_name_in
data_switch_out = "/"
dataset_name_out = "for_temmat_raugh_changed_" + dataset_name_in
data_path_out = data_root + data_switch_out + dataset_name_out

print("")
print("data_path_in:" + data_path_in)
print("data_path_out:" + data_path_out)

cnt_list = [0]*instance_number
print(len(cnt_list))

c = 1
x_data = []
with h5py.File(data_path_in, mode="r") as f:
    for n in range(dataset_number):
        pcl_data = f["data_" + str(n + 1)]['pcl'][()]
        # mask_data = f["data_" + str(n + 1)]['pose'][()]
        # print(mask_data.shape)
        print(n)
        # prepare_data = np.hstack([pcl_data, mask_data])
        print(pcl_data.shape)
        original_data, pcd_offset = getNormalizedPcd(pcl_data, resolution)
        original_data = np.array(original_data)
        x_data.append(original_data)
x_data = np.array(x_data)
print(x_data.shape)

with h5py.File(data_path_out, 'w') as file:
    for i in range(dataset_number):
        file.create_group('data_' + str(i+1))
        occluder_class = 0
        y_data = occluder_class
        file['data_' + str(i+1)].create_dataset('pcl', data=x_data[i, :, :3])
        file['data_' + str(i+1)].create_dataset('class', data=y_data)