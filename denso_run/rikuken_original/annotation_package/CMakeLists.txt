cmake_minimum_required(VERSION 3.0.2)
project(annotation_package)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  message_filters
  color_cloud_bridge
)

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES annotation_package
  CATKIN_DEPENDS 
    pcl_conversions 
    pcl_msgs 
    pcl_ros 
    roscpp 
    rospy 
    sensor_msgs 
    std_msgs 
    tf 
    tf_conversions
    message_filters
  DEPENDS PCL
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(mesh_cloud_publisher src/mesh_cloud_node.cpp src/mesh_kai_cloud.cpp src/mesh_sampler.cpp)
target_link_libraries(mesh_cloud_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(nearest_search src/nearest_search.cpp src/nearest_search_node.cpp)

target_link_libraries(nearest_search
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(mesh_cloud_bara_publisher src/mesh_kai_cloud.cpp src/mesh_sampler.cpp src/mesh_cloud_bara_node.cpp)
target_link_libraries(mesh_cloud_bara_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(nearest_search_bara src/nearest_search.cpp src/nearest_search_bara_node.cpp)
target_link_libraries(nearest_search_bara
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(color_cloud_save src/color_cloud_save.cpp)
target_link_libraries(color_cloud_save
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcl_baikai src/pcl_baikai.cpp)
target_link_libraries(pcl_baikai
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_save_2 src/cloud_save_2.cpp)
target_link_libraries(cloud_save_2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(nearest_search_bara_3 src/nearest_search_3.cpp src/nearest_search_bara_3_node.cpp)
target_link_libraries(nearest_search_bara_3
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcl_2_ba src/pcl_2_ba.cpp)
target_link_libraries(pcl_2_ba
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)