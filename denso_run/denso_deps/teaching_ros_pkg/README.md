# teaching_ros_pkg

1. Save praw files initial and finish assemble state of teaching booth by using phoxi_camera.
2. Read initial state praw.

```sh
$ PhoXiControl
$ roslaunch teaching_booth teaching_booth_with_photoneo.launch
$ roslaunch phoxi_camera phoxi_camera_center_sim.launch 
$ roslaunch teaching_processing get_object.launch object_tf_file:=$(file_name).yaml
$ rosservice call /localize
```

3. Read finish assemble state praw.

```sh
$ PhoXiControl
$ roslaunch teaching_booth teaching_booth_with_photoneo.launch
$ roslaunch phoxi_camera phoxi_camera_center_sim.launch 
$ roslaunch teaching_processing get_object_finish_assemble.launch object_tf_file:=$(file_name).yaml
$ rosservice call /localize
$ roslaunch teaching_processing record_relative_assemble_point.launch assemble_posture_file:=$(file_name).yaml
```

4. Move to Robot booth

```sh
$ PhoXiControl
$ roslaunch denso_bringup vs087_with_tercero_and_photoneo_bringup.launch sim:=false ip_address:=192.168.1.22 planner:=stomp
$ roslaunch phoxi_camera phoxi_camera.launch
$ roslaunch photoneo_localization localization_node.launch
$ rosrun photoneo_localization tf_broadcaster.py
$ rosservice call /localize
$ roslaunch execute_robot work_point_broadcaster.launch assemble_posture_file:=$(file_name).yaml work_position_file:=work_position_real.yaml
$ rosrun execute_robot execute_work
```