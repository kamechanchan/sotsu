cmake_minimum_required(VERSION 2.8.3)
project(denso_state_behavior)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  denso_gazebo_srvs
  denso_recognition_srvs
  denso_state_msgs
  denso_state_srvs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  denso_gazebo_srvs
  denso_state_msgs
  denso_state_srvs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  tf2_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(moving_behavior
  src/moving/moving_behavior.cpp
  src/moving/moving_behavior_node.cpp)
add_dependencies(moving_behavior
  denso_state_msgs_generate_messages_cpp)
add_dependencies(
  moving_behavior
  denso_state_srvs)
target_link_libraries(moving_behavior
  ${catkin_LIBRARIES})

add_executable(measurement_behavior
  src/measurement/measurement_behavior.cpp
  src/measurement/measurement_behavior_node.cpp)
add_dependencies(measurement_behavior
  denso_state_msgs_generate_messages_cpp)
add_dependencies(
  measurement_behavior
  denso_state_srvs)
target_link_libraries(measurement_behavior
  ${catkin_LIBRARIES})

add_executable(measurement_work_point_behavior
  src/measurement/measurement_work_point_behavior.cpp
  src/measurement/measurement_work_point_behavior_node.cpp)
add_dependencies(measurement_work_point_behavior
  denso_state_msgs_generate_messages_cpp)
add_dependencies(
  measurement_work_point_behavior
  denso_state_srvs)
target_link_libraries(measurement_work_point_behavior
  ${catkin_LIBRARIES})

###########
# Install #
###########

set(EXEC_FILES
  moving_behavior
  measurement_behavior
  measurement_work_point_behavior)

set(DIRECTORIES
  launch)

set(PYTHON_SCRIPTS
  scripts/fix_eef_joint_value.py)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install Python scripts
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

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
