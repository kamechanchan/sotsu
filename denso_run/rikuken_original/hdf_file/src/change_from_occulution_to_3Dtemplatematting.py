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
dataset_number = 5000
resolution = 30000
resolution_second = 10000
dataset_name_in = "occulution_kiriwake_11_18_5000_1.hdf5"

data_root = "/home/ericlab/hdf5_data/temmat"
data_switch_in = "/"
data_path_in = data_root + data_switch_in + dataset_name_in
data_switch_out = "/"
dataset_name_out = "for_temmat_occlution_changed_" + dataset_name_in
data_path_out = data_root + data_switch_out + dataset_name_out

print("")
print("data_path_in:" + data_path_in)
print("data_path_out:" + data_path_out)

cnt_list = [0]*instance_number
print(len(cnt_list))

c = 1
x_data = []
cnt = 0
with h5py.File(data_path_in, mode="r") as f:
    for n in range(dataset_number):
        pcl_data = f["data_" + str(n + 1)]['Points'][()]
        mask_data = f["data_" + str(n + 1)]['masks'][()]
        # print(mask_data.shape)
        print(n)
        prepare_data = np.hstack([pcl_data, mask_data])
        print(prepare_data.shape)
        original_data, pcd_offset = getNormalizedPcd_seg(prepare_data, resolution)
        original_data = np.array(original_data)
        occludy_data = []
        # print(original_data.shape)
        # cnt = -1
        for i in range(resolution):
            # cnt += 1
            # print(original_data[i, 3])
            # print(i)
            # if (original_data[i, 3] >= 0) and (original_data[i, 3] < 49):
            if original_data[i, 3] >= 50 and (original_data[i, 3] < 200):
                # original_data[i, 3] = 0
                # cnt -= 1
                # print(i)
                original_data[i, 3] = 1
                # print("af")
                occludy_data.append(original_data[i])

            # elif (original_data[i, 3] >= 49) and (original_data[i, 3] < 200):
            #     original_data[i, 3] = 1
            
            # else:
            #     original_data[cnt, 3] = 2
            #     # cnt -= 1
            #     # print(i)
        occludy_data = np.array(occludy_data)
        if occludy_data.shape[0] == 0:
            print("nothing occludy data!")
            cnt += 1
        else :
            print(occludy_data.shape)
            final_data, pcd_offset = getNormalizedPcd_seg(occludy_data, resolution_second)
            x_data.append(final_data)
x_data = np.array(x_data)
print(x_data.shape)

with h5py.File(data_path_out, 'w') as file:
    for i in range(dataset_number-cnt):
        file.create_group('data_' + str(i+1))
        occludy_class = 1
        y_data = occludy_class
        file['data_' + str(i+1)].create_dataset('pcl', data=x_data[i, :, :3])
        file['data_' + str(i+1)].create_dataset('class', data=y_data)