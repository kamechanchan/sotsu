#! /bin/bash


echo "Change repo to $1 !!"

sros
roscd denso_bringup
git checkout $1

roscd denso_gazebo
git checkout $1

roscd denso_moveit
git checkout $1

roscd denso_control
git checkout $1

roscd denso_common
git checkout $1


