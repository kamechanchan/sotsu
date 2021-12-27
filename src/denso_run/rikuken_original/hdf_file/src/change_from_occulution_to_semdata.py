from types import prepare_class
import h5py
import numpy as np
from numpy.testing._private.utils import print_assert_equal


def getNormalizedPcd_seg(np_cloud, resolution):
    pcd_offset = np.expand_dims(np.mean(np_cloud[:,:3], axis=0), 0)
    pre_pcd_data = np_cloud[:,:3] - pcd_offset  #original
    # pcd_offset = np.expand_dims(np.mean(np_cloud, axis=0), 0)
    # pre_pcd_data = np_cloud - pcd_offset  #original
    mask_data = np_cloud[:,3]
    # mask_data = cloud_data[:,3]
    mask_data = np.expand_dims(mask_data, 1)
    pcd_data = np.hstack([pre_pcd_data, mask_data])
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
dataset_number = 630
resolution = 40000
resolution_second = 10000
dataset_name_in = "occulution_kiriwake_9_14_1000_1.hdf5"

data_root = "/home/ericlab/Downloads/"
data_switch_in = "/"
data_path_in = data_root + data_switch_in + dataset_name_in
data_switch_out = "/"
dataset_name_out = "semantic_changed_" + dataset_name_in
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
        pcl_data = f["data_" + str(n + 1)]['Points'][()]
        mask_data = f["data_" + str(n + 1)]['masks'][()]
        # print(mask_data.shape)
        print(n)
        prepare_data = np.hstack([pcl_data, mask_data])
        original_data, pcd_offset = getNormalizedPcd_seg(prepare_data, resolution)
        original_data = np.array(original_data)
        # print(original_data.shape)
        cnt = -1
        for i in range(resolution):
            cnt += 1
            # print(original_data.shape)
            # print(i)
            if (original_data[i, 3] >= 0) and (original_data[i, 3] < 49):
                original_data[i, 3] = 0
                # cnt -= 1
                # print(i)

            elif (original_data[i, 3] >= 49) and (original_data[i, 3] < 200):
                original_data[i, 3] = 1
            
            else:
                original_data[cnt, 3] = 2
                # cnt -= 1
                # print(i)
        print(original_data.shape)
        final_data, pcd_offset = getNormalizedPcd_seg(original_data, resolution_second)
        x_data.append(final_data)
x_data = np.array(x_data)
print(x_data.shape)

with h5py.File(data_path_out, 'w') as file:
    for i in range(dataset_number):
        file.create_group('data_' + str(i+1))
        # for j in range(resolution_second):
        pre_data = x_data[i, :, 3]
        mask_data = np.array(pre_data)
        mask_data = mask_data[:,np.newaxis]
        file['data_' + str(i+1)].create_dataset('Points', data=x_data[i, :, :3])
        file['data_' + str(i+1)].create_dataset('masks', data=mask_data)