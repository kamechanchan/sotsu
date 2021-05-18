cmake_minimum_required(VERSION 2.8.3)
project(template_matching)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    sensor_msgs
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# Python scripts
set(PYTHON_SCRIPTS
  scripts/template_matching.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

