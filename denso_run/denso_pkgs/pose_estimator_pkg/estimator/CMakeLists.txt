cmake_minimum_required(VERSION 2.8.3)
project(estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  color_cloud_bridge
)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(color_pointcloud src/color_pointcloud.cpp)
target_link_libraries(color_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}

)

###########
# Install #
###########


set(DIRECTORIES
  launch)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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
