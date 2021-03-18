# pose_estimator_pkg in PyTorch

## Overview
ROS pose estimation packages for Denso robots

## Install
See [denso_apps](https://github.com/Nishida-Lab/denso_apps)

# Getting Started

### Installation
- Clone this repo:
 ```
 $ git clone https://github.com/maeshu/pose_estimator_pkg.git
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

### Training
Run training
 ```
 $ cd trainer
 $ bash scripts/${arch}/train.sh
 ```
To view the training loss plots, in another terminal run ```tensorboard --logdir runs``` and click [http://localhost:6006](http://localhost:6006).



## CI
See [here](https://github.com/Nishida-Lab/denso_docs/tree/master/ci) for detail decumentation.

Replace the repository specific keywords in the above link as follows:
- `<your_repo>` -> `denso_recognition`
- `<your_pkg>` -> `pointcloud_processing`, `ar_marker_urdf`
- `<your_rosinstall_dir>` -> `.`

## Reference
- STL to mesh cloud
  - https://github.com/GT-RAIL/rail_mesh_icp
