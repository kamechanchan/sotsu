# denso_state_machine

[![](https://github.com/Nishida-Lab/denso_state_machine/workflows/CI/badge.svg)](https://github.com/Nishida-Lab/denso_state_machine/actions)

## Overview
ROS state machine related packages for Denso robots

## Install

### Install with denso_apps

See [denso_apps](https://github.com/Nishida-Lab/denso_apps)

### Install without denso_apps

```bash
cd your_ws/src
git clone https://github.com/Nishida-Lab/denso_state_machine.git
rosdep install -iry --from-paths .
catkin build
```

## Usage

### Behavior

The behavior of each state is implemented as ROS service.
To bringup each behavior, execute folloing command.

```bash
roslaunch denso_state_behavior state_behaviors_bringup.launch
```

#### args
- exe_speed_rate (default: 1.0)
  - Robot moving speed magnification
  - For example, setting it to 2.0 makes the speed of movement twice faster.

#### State:moving
- `/state_behavior/moving` (type:`denso_state_srvs/Moving`)
  - When a target point is specified at `<denso_state_srvs/Moving>.target_pose` as type of `geometry_msgs/Pose` and call service, the robot moves to the target point.

## CI
See [here](https://github.com/Nishida-Lab/denso_docs/tree/master/ci) for detail decumentation.

Replace the repository specific keywords in the above link as follows:

- `<your_repo>` -> `denso_state_machine`
- `<your_pkg>` -> `denso_state_behavior`
- `<your_rosinstall_dir>` -> Nothing
