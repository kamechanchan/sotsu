# 3d_recognition_ros
対象物体周辺の点群を切り取り、その切り取った点群をCNNに入力することで対象物体の位置・姿勢を推定。最終補正にICPを使用。

## Requirements
- ubuntus 16.04
- PhoXiControl
- ROS kinetic
- chainer 4.0.0
- photoneo_localization
```
$ python -m pip install --user scipy
$ python -m pip install --user chainer==4.0.0
$ python -m pip install --user 'cupy-cuda90>=6.3.0,<7.0.0'
$ python -m pip install --user h5py
```
or
```
$ pip install --user scipy
$ pip install --user chainer==4.0.0
$ pip install --user 'cupy-cuda90>=6.3.0,<7.0.0'
$ pip install --user h5py
```

### Usage
#### 0.open 7 terminal and connect pass in the all terminal

#### 1.roscore
```
$ roscore
```
#### 2.PhoXiController(Connect to the photoneo sensor system (online/offline))
```
$ PhoXiControl
```
drag & drop praw file (left_down_0_dense.praw)
SoftTrig` : single shot
`SoftwareTriger`: continuous shot
`save pointcloud`: `.praw` format can be used offline in the localization SDK

#### 3.Sensor calibration and publish point cloud
```
$ roslaunch denso_bringup vs087_with_mhand2_and_photoneo_bringup.launch
$ roslaunch phoxi_camera phoxi_camera_center_sim.launch
```

#### 4.Voxelization point cloud
```
$ roslaunch cnn_pose_estimator make_input.launch division_number:=70 source_pc:=/photoneo_left/pointcloud sensor_frame_id:=photoneo_left_optical_frame
```
or
```
$ roslaunch cnn_pose_estimator euclidian_cluster.launch
$ roslaunch cnn_pose_estimator voxelization_from_cluster.launch
```

#### 5.Estimate 6 DOF of the target object to input voxels to 3D-CNN(if you DON'T use gpu, input -g -1)
```
$ rosrun cnn_pose_estimator 3d_recognition.py -g 0 -d 70 -m hv8_70_rotation_regular_random.model -s photoneo_left_optical_frame
```

#### 6.registration using ICP algorithm
```
$ roslaunch cnn_pose_estimator registration_cloud.launch model_name:="hv8" registrated_frame_id:="HV8" sensor_frame_id:=photoneo_left_optical_frame scene_plane_threshold:=0.0015
```

#### 7.broadcasting static tf of registrated model
```
$ rosrun photoneo_localization tf_broadcaster_cnn_recognition.py
```


### Usage(launch estimate pose server)
#### 0.open 7 terminal and connect pass in the all terminal

#### 1.roscore
```
$ roscore
```
#### 2.PhoXiController(Connect to the photoneo sensor system (online/offline))
```
$ PhoXiControl
```
drag & drop praw file (left_down_0_dense.praw)
SoftTrig` : single shot
`SoftwareTriger`: continuous shot
`save pointcloud`: `.praw` format can be used offline in the localization SDK

#### 3.Sensor calibration and publish point cloud
```
$ roslaunch denso_bringup vs087_with_mhand2_and_photoneo_bringup.launch
$ roslaunch phoxi_camera phoxi_camera_center_sim.launch
```

#### 4.Launch pose estimate server
```
$ roslaunch cnn_pose_estimator estimate_pose_server_node.launch scene_topic_name:=/cropped_cloud model_plane_threshold:=0.00001 scene_plane_threshold:=0.00001 leaf_model:=0.005 leaf_scene:=0.01
```

#### 5.Make input bulk cloud
```
$ roslaunch pointcloud_processing crop_cloud.launch crop_x_min:=0.12 crop_x_max:=0.37 crop_y_min:=0.18 crop_y_max:=0.35 crop_z_min:=-0.353 crop_z_max:=-0.1
```

#### 6.Estimate target object pose
```
$ rosservice call /cnn_pose_estimator/estimate_object_pose
```
