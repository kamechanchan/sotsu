#### やり方的には最初に石山くんのワークスペース(src/)に移動して次を実行
```
git clone https://github.com/ERiC-Labo/2d_to_3d
catkin build 2d_to_3d_ros
```

#### 次のコマンドを実行
```
roslaunch tf_publish spawn_object.launch
```
```
roslaunch 2d_to_3d_ros colored_cloud.launch