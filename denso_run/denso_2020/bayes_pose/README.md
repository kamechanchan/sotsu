# bayes_pose

[![](https://github.com/AtsukiYokota/bayes_pose/workflows/CI/badge.svg)](https://github.com/AtsukiYokota/bayes_pose/actions)

6D Pose Estimation using Bayesian Optimization

## dependency
- [bayes_opt](https://github.com/rmcantin/bayesopt)
- [meshcloud_publisher_pkg](https://github.com/Nishida-Lab/meshcloud_publisher_pkg)

## コマンド

### ICP

```
roslaunch denso_bringup vs087_with_tercero_and_photoneo_bringup.launch
roslaunch bayes_pose spawn_object.launch model_name:=hv8
roslaunch pointcloud_processing crop_cloud.launch crop_pc_src:=/photoneo_center/sensor/points crop_x_min:=0.15 crop_z_min:=0.001 crop_z_max:=0.1
roslaunch meshcloud_publisher_pkg meshcloud_publisher.launch yaml_file:=hv8.yaml leaf_size:=0.001 sampling_points:=50000
roslaunch bayes_pose icp_registrator.launch object_frame_name:=HV8_00
```

### bayes_pose
```
roslaunch denso_bringup vs087_with_tercero_and_photoneo_bringup.launch
roslaunch bayes_pose spawn_object.launch model_name:=hv8
roslaunch pointcloud_processing crop_cloud.launch crop_pc_src:=/photoneo_center/sensor/points crop_x_min:=0.15 crop_z_min:=0.001 crop_z_max:=0.1
roslaunch meshcloud_publisher_pkg meshcloud_publisher.launch yaml_file:=hv8.yaml leaf_size:=0.001 sampling_points:=50000
roslaunch bayes_pose bayes_registrator.launch object_frame_name:=HV8_00
```
