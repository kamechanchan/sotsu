cmake_minimum_required(VERSION 2.8.3)
project(oneshot_calibration_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
# set(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  tf2_ros
  tf_conversions
)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    sensor_msgs
    tf
    tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

add_executable(ar_center_broadcaster
  src/ar_center_broadcaster.cpp
  src/ar_center_broadcaster_node.cpp)
target_link_libraries(ar_center_broadcaster
  ${catkin_LIBRARIES})

add_executable(true_ar_center_broadcaster
  src/true_ar_center_broadcaster.cpp
  src/true_ar_center_broadcaster_node.cpp)
target_link_libraries(true_ar_center_broadcaster
  ${catkin_LIBRARIES})

add_executable(ar_oneshot_calibrator
  src/ar_oneshot_calibrator.cpp
  src/ar_oneshot_calibrator_node.cpp)
target_link_libraries(ar_oneshot_calibrator
  ${catkin_LIBRARIES})

add_executable(ar_cube_pointcloud_extractor
  src/ar_cube_pointcloud_extractor.cpp
  src/ar_cube_pointcloud_extractor_node.cpp)
target_link_libraries(ar_cube_pointcloud_extractor
  ${catkin_LIBRARIES}
)
add_executable(workspace_pointcloud_extractor
  src/workspace_pointcloud_extractor.cpp
  src/workspace_pointcloud_extractor_node.cpp)
target_link_libraries(workspace_pointcloud_extractor
  ${catkin_LIBRARIES}
)
add_executable(icp_calibrator
  src/icp_calibrator.cpp
  src/icp_calibrator_node.cpp)
target_link_libraries(icp_calibrator
  ${catkin_LIBRARIES}
)
add_executable(pc_calib_broadcaster
  src/pc_calib_broadcaster.cpp
  src/pc_calib_broadcaster_node.cpp)
target_link_libraries(pc_calib_broadcaster
  ${catkin_LIBRARIES})
add_executable(hand_ar_oneshot_calibrator
  src/hand_ar_oneshot_calibrator.cpp
  src/hand_ar_oneshot_calibrator_node.cpp
  )
target_link_libraries(hand_ar_oneshot_calibrator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(handmarker_pointcloud_extractor
  src/handmarker_pointcloud_extractor.cpp
  src/handmarker_pointcloud_extractor_node.cpp
  )

target_link_libraries(handmarker_pointcloud_extractor
  ${catkin_LIBRARIES})
###########
# Install #
###########

set(EXEC_FILES
  ar_center_broadcaster
  true_ar_center_broadcaster
  ar_oneshot_calibrator
  ar_cube_pointcloud_extractor
  workspace_pointcloud_extractor
  icp_calibrator
  pc_calib_broadcaster
  hand_ar_oneshot_calibrator
  handmarker_pointcloud_extractor
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
  ./config/
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

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
