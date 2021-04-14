#!/bin/sh

cd ../../../

# install dependent non-apt packages via wstool
wstool init src
wstool merge -t src src/denso_apps/.rosinstalls/.kinetic_depends_use_ssh.rosinstall
wstool update -t src

# install dependent apt packages via wstool
rosdep init
rosdep update
rosdep install -i -r -y --from-paths src --ignore-src

# build
catkin build
source devel/setup.bash
rospack profile

# test
catkin run_tests
catkin_test_results
