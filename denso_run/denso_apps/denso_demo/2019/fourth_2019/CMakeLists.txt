cmake_minimum_required(VERSION 2.8.3)
project(fourth_2019)

find_package(catkin REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
set(PYTHON_SCRIPTS
  scripts/t_pipe_pick_and_place.py
)
