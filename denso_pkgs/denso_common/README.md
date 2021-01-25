# denso_common

[![](https://github.com/Nishida-Lab/denso_common/workflows/CI/badge.svg)](https://github.com/Nishida-Lab/denso_common/actions)

## Overview
ROS commonly used packages in both real and virtual for Denso robots

## Install
See [denso_apps](https://github.com/Nishida-Lab/denso_apps)

## Usage

### Parallel planner and executer

#### launch

#### args
- exe_speed_rate (default:1.0)
  - Robot moving speed magnification
  - For example, setting it to 2.0 makes the speed of movement twice faster.

```bash
roslaunch denso_execute parallel_executer.launch exe_speed_rate:=1.0
```

#### Plannning request
Publish planning request as denso_execute/PlanningRequest type to topic shown below
```
/planning_request
```
For example,
```bash
rostopic pub /planning_request denso_execute/PlanningRequest "request:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'world'
  pose:
    position:
      x: 0.5
      y: 0.0
      z: 0.5
    orientation:
      x: 1.0
      y: 0.0
      z: 0.0
      w: 0.0
grasp: false"
```

- `header` follows [std_msgs/Header Message](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Header.html)
- `pose` follows [geometry_msgs/Pose Message](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html)

## CI
See [here](https://github.com/Nishida-Lab/denso_docs/tree/master/ci) for detail decumentation.

Replace the repository specific keywords in the above link as follows:
- `<your_repo>` -> `denso_common`
- `<your_pkg>` -> `denso_execute`, `denso_descriptions`
- `<your_rosinstall_dir>` -> `.`
