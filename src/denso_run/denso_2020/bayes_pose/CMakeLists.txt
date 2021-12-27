cmake_minimum_required(VERSION 2.8.3)
project(bayes_pose)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  tf
  tf2
  tf2_eigen
  tf2_ros
)

find_package(PCL REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    pcl_ros
    roscpp
    std_msgs
    sensor_msgs
    tf
    tf2
    tf2_eigen
    tf2_ros
  DEPENDS PCL
)
include_directories(
  include
  /usr/local/include # bayesopt
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(/usr/local/lib) # bayesopt

###########
## Build ##
###########

add_executable(c2c_distance
  src/c2c_distance.cpp
  src/c2c_distance_node.cpp)
target_link_libraries(c2c_distance
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(icp_registrator
  src/icp_registrator.cpp
  src/icp_registrator_node.cpp)
target_link_libraries(icp_registrator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

# add_executable(bayes_registrator
#   src/bayes_registrator.cpp
#   src/bayes_registrator_node.cpp)
# target_link_libraries(bayes_registrator
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   bayesopt
#   nlopt)

add_executable(sim_odom_tf_converter
  src/sim_odom_tf_converter.cpp)
target_link_libraries(sim_odom_tf_converter
  ${catkin_LIBRARIES})
  # ${PCL_LIBRARIES})

#############
## Install ##
#############

set(EXEC_FILES
  sim_odom_tf_converter
  c2c_distance
  icp_registrator
  # bayes_registrator
)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Set directories
set(DIRECTORIES
  launch/
  model/
  config/
)

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
