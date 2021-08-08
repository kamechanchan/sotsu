import h5py


cnt = 0
c = 1
with h5py.File('/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/annotation_package/dataset/tsuchida/instance_changed_1526.hdf5', mode="r") as f:
    for i in f["data_1"].values():
        # cnt +=1
        # c += 1
        # if cnt == 5000:
        print(i)
            # cnt =0

