cmake_minimum_required(VERSION 2.8.3)
project(gen_dataset)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  pose_estimator_msgs
  roscpp
  roslib
  sensor_msgs
  tf
  tf2
  tf2_ros
)

find_package(PCL REQUIRED)





catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    pose_estimator_msgs
    roscpp
    sensor_msgs
    tf2_ros
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_executable(
  make_voxel_data
  src/make_voxel_data.cpp
  src/make_voxel_data_node.cpp
)

add_executable(
  record_euler
  src/record_euler.cpp
  src/record_euler_node.cpp
)

add_executable(
  cloud_downsampler
  src/cloud_downsampler.cpp
)

add_executable(
  cloud_planar_segmenter
  src/cloud_planar_segmenter.cpp
)

add_executable(
  dnn_input
  src/dnn_input.cpp
  src/dnn_input_node.cpp
)


target_link_libraries(
  make_voxel_data
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  record_euler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  cloud_downsampler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  cloud_planar_segmenter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(
  dnn_input
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


#############
## Install ##
#############

# Exec Files
set(EXEC_FILES
  record_euler
  cloud_downsampler
  cloud_planar_segmenter
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
  scripts/random_move_euler.py
  scripts/open3d_3d_operator.py
  )

#foreach(scripts ${PYTHON_SCRIPTS})
#  catkin_install_python(PROGRAMS ${scripts}
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#endforeach()

# Shell scripts
set(SHELL_SCRIPTS
  scripts/delete_ground.sh
  scripts/zero_gravity.sh
  )

# Install shell scripts
foreach(scripts ${SHELL_SCRIPTS})
  install(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Other directories
set(DIRECTORIES
  ./launch/
  ./rviz/
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
