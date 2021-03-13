cmake_minimum_required(VERSION 2.8.3)
project(cnn_pose_estimator)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  denso_recognition_msgs
  denso_recognition_srvs
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  phoxi_camera_srvs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    denso_recognition_msgs
    denso_recognition_srvs
    eigen_conversions
    geometry_msgs
    pcl_msgs
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(
  make_input_node
  src/make_input.cpp
  src/make_input_node.cpp
  )

add_executable(
  make_input_server_node
  src/make_input_server.cpp
  src/make_input_server_node.cpp
  )

add_executable(
  euclidean_cluster_node
  src/euclidean_cluster.cpp
  src/euclidean_cluster_node.cpp
  )

add_executable(
  voxelization_from_cluster_node
  src/voxelization_from_cluster.cpp
  src/voxelization_from_cluster_node.cpp
  )

add_executable(
  registration_cloud_node
  src/registration_cloud.cpp
  src/registration_cloud_node.cpp
  )

add_executable(
  validation_registration_node
  src/validation_registration.cpp
  src/validation_registration_node.cpp
  )

add_executable(
  registration_pose_server
  src/registration_pose_server.cpp
  src/registration_pose_server_node.cpp
)

add_executable(
  inverse_pose_server
  src/inverse_pose_server.cpp
  src/inverse_pose_server_node.cpp
)

add_executable(
  estimate_object_pose_server
  src/estimate_object_pose_server.cpp
  src/estimate_object_pose_server_node.cpp
)

target_link_libraries(
  make_input_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  make_input_server_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  euclidean_cluster_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  voxelization_from_cluster_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  registration_cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  validation_registration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  registration_pose_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  inverse_pose_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  estimate_object_pose_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  estimate_object_pose_server
  euclidean_cluster_node
  inverse_pose_server
  make_input_node
  make_input_server_node
  registration_cloud_node
  voxelization_from_cluster_node
  )

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Python scripts
set(PYTHON_SCRIPTS
  scripts/3d_recognition.py
  scripts/network_euler.py
  scripts/network_matrix.py
  scripts/object_pose_estimator.py
  scripts/object_pose_estimator_server.py
  scripts/visualize.py
  scripts/visualize_voxel.py
)

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./launch/
  ./pcds/
  )

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
