cmake_minimum_required(VERSION 2.8.3)
project(meshcloud_publisher)

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
)

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
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

add_executable(meshcloud_pub
  src/meshcloud_publisher.cpp
  src/meshcloud_publisher_node.cpp
  src/meshcloud_sampler.cpp
  )

add_executable(hand_meshcloud_pub
  src/handmarker_meshcloud_pub.cpp
  src/handmarker_meshcloud_pub_node.cpp
  src/meshcloud_sampler.cpp
  )
target_link_libraries(meshcloud_pub pcl_common pcl_io
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES}
  )

target_link_libraries(hand_meshcloud_pub pcl_common pcl_io
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES}
  )
###########
# Install #
###########

set(EXEC_FILES
  meshcloud_pub
  hand_meshcloud_pub
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
