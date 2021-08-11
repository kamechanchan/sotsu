cmake_minimum_required(VERSION 2.8.3)
project(jacobi_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
#SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  )


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  )

#[[
## src dir ##
add_executable(generate_opt_path src/generate_opt_path.cpp)
target_link_libraries(generate_opt_path ${catkin_LIBRARIES})


###########
# Install #
###########

set(EXEC_FILES
  generate_opt_path
  )

set(DIRECTORIES
 ./config/
 ./launch/
  )


# Install executables
foreach(nodes ${EXEC_FILES})
 install(TARGETS ${nodes}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

#Install include directory
install(DIRECTORY include/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#Install all files
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
]]