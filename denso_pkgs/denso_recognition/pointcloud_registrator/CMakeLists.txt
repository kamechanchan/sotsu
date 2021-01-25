cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_registrator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    eigen_conversions
    pcl_ros
    roscpp
    sensor_msgs
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

add_executable(${PROJECT_NAME}
  src/pointcloud_registrator.cpp
  src/pointcloud_registrator_node.cpp)
target_link_libraries(${PROJECT_NAME} pcl_common pcl_io
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  )

###########
# Install #
###########

set(EXEC_FILES
  ${PROJECT_NAME}
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
