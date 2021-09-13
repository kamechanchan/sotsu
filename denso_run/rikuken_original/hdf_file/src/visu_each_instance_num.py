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
dataset_number = 2000
resolution = 1024

cnt_list = [0]*instance_number
print(len(cnt_list))

cnt = 0
c = 1
x_data = []
with h5py.File('/home/ericlab/DENSO_results/August/pcl_visu/progress_output/instance_segmentation/instance_changed_8_7_1526.hdf5+instance_changed_instance_tsuchida_8_12_500_1.hdf5+instance_changedinstance_tsuchida_8_11_1000_1.hdf5/5/result5.hdf5', mode="r") as f:
    for n in range(dataset_number):
        pcl_data = f["data_" + str(n + 1)]['Points'][()]
        mask_data = f["data_" + str(n + 1)]['ground_truth'][()]
        # print(mask_data.shape)
        print(n)
        prepare_data = np.hstack([pcl_data, mask_data])
        original_data, pccd_offset = getNormalizedPcd_seg(prepare_data, resolution)
        x_data.append(original_data)
        # print(original_data.shape)
        for i in range(resolution):
            for j in range(instance_number):
                if original_data[i,3] == j:
                    cnt_list[j] += 1
x_data = np.array(x_data)
print(x_data.shape)
print(cnt_list)