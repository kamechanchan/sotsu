# pose_estimator_pkg in PyTorch

## Overview
ROS pose estimation packages for Denso robots


# Getting Started

### Installation
- See [denso_apps](https://github.com/Nishida-Lab/denso_apps)
- Clone this repo:
 ```
 $ git clone https://github.com/maeshu/pose_estimator_pkg.git
 ```
- Install Dependencies
 ```
 $ pip2 install -r requirements.txt
 ```

### Download datasets
Two options are available 

- Download datasets from Dropbox: 
 ```
 $ cd trainer
 $ bash scripts/${arch}/get_data.sh
 ```
 
 New directory you stated in ${arch}is created under datasets dir


- Make datasets on your own 
```
$ cd datasets
$ mkdir {$arch}
```

put on your datasets on the directory.
Note, *.hdf5 format file is only available.

### Make Dataset on your own
Run generate_dataset
 ```
 $ cd gen_dataset
 ```
 
  ```
 $ roslaunch gen_dataset gen_dataset.launch num_dataset:={$num_dataset}
 ```
 - `{$num_dataset}` -> dataset number
 

### Training
Run training
 ```
 $ cd trainer
 $ bash scripts/${arch}/train.sh
 ```
 In case of using arch type, "3DCNN", you have to specify resolution arg like below.
  ```
 $ bash scripts/3DCNN/train.sh 50
 ```
 
To view the training loss plots, in another terminal run ```tensorboard --logdir runs``` and click [http://localhost:6006](http://localhost:6006).

### Estimate Pose
Run estimator
 ```
 $ cd estimator
 ```
 In case of using arch type, "3DCNN"


### Tips 

- when you encountering trouble related to phoxi_camera_srvs, please change your branch to add_get_one_frame_srv in phoxi_ros_pkg.

## CI
See [here](https://github.com/Nishida-Lab/denso_docs/tree/master/ci) for detail decumentation.

Replace the repository specific keywords in the above link as follows:
- `<your_repo>` -> `denso_recognition`
- `<your_pkg>` -> `pointcloud_processing`, `ar_marker_urdf`
- `<your_rosinstall_dir>` -> `.`

## Reference
- PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation
  - https://arxiv.org/abs/1612.00593
  
- PointNet++: Deep Hierarchical Feature Learning on Point Sets in a Metric Space
  - https://papers.nips.cc/paper/7095-pointnet-deep-hierarchical-feature-learning-on-point-sets-in-a-metric-space.pdf
  
- Deep Sets
  - https://arxiv.org/abs/1703.06114

- STL to mesh cloud
  - https://github.com/GT-RAIL/rail_mesh_icp
