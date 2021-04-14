cmake_minimum_required(VERSION 3.0.2)
project(2d_to_3d_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 2d_to_3d_ros
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport pcl_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(colord_cloud src/colord_cloud.cpp)
target_link_libraries(colord_cloud
  ${catkin_LIBRARIES}
)