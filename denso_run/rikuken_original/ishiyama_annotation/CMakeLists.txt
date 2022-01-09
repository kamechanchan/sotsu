cmake_minimum_required(VERSION 3.0.2)
project(ishiyama_annotation)
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
  estimator
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


add_executable(ishiyama_img src/ishiyama_img.cpp)
target_link_libraries(ishiyama_img
  ${catkin_LIBRARIES}
  
)

add_executable(nearest_search_ishiyama_img src/nearest_search_ishiyama_img.cpp src/nearest_search_ishiyma_node.cpp)
target_link_libraries(nearest_search_ishiyama_img
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(mesh_cloud_bara_publisher src/mesh_kai_cloud.cpp src/mesh_sampler.cpp src/mesh_cloud_bara_node.cpp)
target_link_libraries(mesh_cloud_bara_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(pcl_baikai src/pcl_baikai.cpp)
target_link_libraries(pcl_baikai
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(quaternion src/quartenion_convert.cpp)
target_link_libraries(quaternion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcl_baikai_debug src/pcl_baikai_debug.cpp)
target_link_libraries(pcl_baikai_debug
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)