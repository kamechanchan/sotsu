cmake_minimum_required(VERSION 3.0.2)
project(pose_estimator_measure)

find_package(catkin REQUIRED)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  crop_cloud_node
  src/crop_cloud.cpp
  src/crop_cloud_node.cpp
  )

target_link_libraries(
  crop_cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  crop_cloud_node
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
  scripts/grand_truth_broadcaster.py
)

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./launch/
  ./test_pcd_dataset/
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
