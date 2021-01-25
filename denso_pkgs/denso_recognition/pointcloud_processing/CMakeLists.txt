cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_processing)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  denso_recognition_srvs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    denso_recognition_srvs
    pcl_conversions
    pcl_msgs
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

###########
## Build ##
###########

add_executable(crop_cloud_node
  src/crop_cloud.cpp
  src/crop_cloud_node.cpp)
target_link_libraries(crop_cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(crop_cloud_server_node
  src/crop_cloud_server.cpp
  src/crop_cloud_server_node.cpp)
target_link_libraries(crop_cloud_server_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(filter
  src/filter.cpp
  src/filter_node.cpp)
target_link_libraries(filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(downsampling
  src/downsampling.cpp
  src/downsampling_node.cpp)
target_link_libraries(downsampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})


###########
# Install #
###########

set(EXEC_FILES
  crop_cloud_node
  crop_cloud_server_node
  filter
  downsampling
  )
# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(DIRECTORIES
  ./launch/
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

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()
