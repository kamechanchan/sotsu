cmake_minimum_required(VERSION 2.8.3)
project(target_extractor)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  denso_recognition_srvs
  pcl_conversions
  geometry_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    denso_recognition_srvs
    pcl_ros
    roscpp
    sensor_msgs
    std_srvs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(bulk_extractor
  src/bulk_extractor.cpp
  src/bulk_extractor_node.cpp)

add_executable(top_part_extractor
  src/top_part_extractor.cpp
  src/top_part_extractor_node.cpp)

add_executable(set_initial_cloud
  src/set_initial_cloud.cpp
  src/set_initial_cloud_node.cpp)

add_executable(region_growing_extractor_server
  src/region_growing_extractor_server.cpp
  src/region_growing_extractor_server_node.cpp)

target_link_libraries(bulk_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

target_link_libraries(top_part_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

target_link_libraries(set_initial_cloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

target_link_libraries(region_growing_extractor_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

#############
## Install ##
#############

set(EXEC_FILES
  bulk_extractor
  region_growing_extractor_server
  set_initial_cloud
  top_part_extractor
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
  ./pcd/
  ./config/
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
