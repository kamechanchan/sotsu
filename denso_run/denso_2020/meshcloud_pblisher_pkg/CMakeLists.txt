cmake_minimum_required(VERSION 3.0.2)
project(meshcloud_publisher_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  tf
  tf2_ros
)

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros
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

add_executable(meshcloud_publisher
  src/meshcloud_publisher.cpp
  src/meshcloud_publisher_node.cpp
  src/meshcloud_sampler.cpp
  )

target_link_libraries(meshcloud_publisher
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} yaml-cpp
  )

###########
# Install #
###########

set(EXEC_FILES
  meshcloud_publisher
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
  config/
  launch/
  meshes/
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
