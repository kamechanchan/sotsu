cmake_minimum_required(VERSION 3.0.2)
project(w2d_to_3d_ros)

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
  message_filters
  message_generation
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  color_cloud_bridge
)
add_message_files(
  FILES
  shinya.msg 
  hayasa.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES w2d_to_3d_ros
  CATKIN_DEPENDS cv_bridge image_geometry image_transport pcl_ros roscpp sensor_msgs tf message_filters message_runtime std_msgs tf2_ros tf2_geometry_msgs tf2
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(colord_cloud src/colord_cloud.cpp)
target_link_libraries(colord_cloud
  ${catkin_LIBRARIES}
)

add_executable(message_publish src/message_publish.cpp)
add_dependencies(message_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(message_publish
  ${catkin_LIBRARIES}
)

add_executable(another_message_publish src/anothre_message_publish.cpp)
add_dependencies(another_message_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(another_message_publish
  ${catkin_LIBRARIES}
)

add_executable(colored_pointcloud src/colored_pointcloud.cpp src/colored_pointcloud_node.cpp)
target_link_libraries(colored_pointcloud
  ${catkin_LIBRARIES}
)

add_executable(ano_colored_pointcloud src/ano_colored_pointcloud.cpp src/ano_colored_pointcloud_node.cpp)
target_link_libraries(ano_colored_pointcloud
  ${catkin_LIBRARIES}
)

add_executable(angle_trans src/angle_transform.cpp)
target_link_libraries(angle_trans
  ${catkin_LIBRARIES}
)

add_executable(exec_yolo src/exec_yolo.cpp src/exec_yolo_node.cpp)
target_link_libraries(exec_yolo
  ${catkin_LIBRARIES}
)