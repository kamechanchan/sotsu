cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_usage)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_ros
  pcl_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)

link_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_usage
  CATKIN_DEPENDS eigen_conversions geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_save src/cloud_save.cpp)
target_link_libraries(cloud_save
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_express src/cloud_express.cpp)
target_link_libraries(cloud_express
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_sonomama src/cloud_sonomama.cpp)
target_link_libraries(cloud_sonomama
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(cloud_downsample src/cloud_downsampling.cpp)
target_link_libraries(cloud_downsample
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)