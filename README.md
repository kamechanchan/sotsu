
```
sudo apt install ros-noetic-ros-numpy


sudo apt install pcl-tools


sudo apt install -y libpcl-dev


sudo apt install -y python3-pip

sudo apt install -y python3-pcl
sudo apt install -y python3-h5py
pip3 install open3d
sudo apt install python3-catkin-tools
sudo apt install python3-catkin-pkg
sudo apt install python3-osrf-pycommon
pip3 install torch==1.7.1+cpu torchvision==0.8.2+cpu torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html


cd ~/ros_package/denso_ws 
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Estimation Test 
```
roslaunch tf_publish spawn_object.launch 

roslaunch denso_gazebo model_tf_broadcaster.launch

roslaunch cloud_practice planar_segmentation.launch

roslaunch estimator pose_estimator.launch

roslaunch tf_publish error_calculate.launch
```

### Getting dataset
```
roslaunch tf_publish spawn_object.launch 

roslaunch denso_gazebo model_tf_broadcaster.launch

roslaunch cloud_practice planar_segmentation.launch

roslaunch gen_dataset record_data.launch
```

### Getting dataset
```
roslaunch tf_publish spawn_object.launch 

roslaunch denso_gazebo model_tf_broadcaster.launch

roslaunch cloud_practice planar_segmentation.launch

roslaunch gen_dataset record_data.launch
```
