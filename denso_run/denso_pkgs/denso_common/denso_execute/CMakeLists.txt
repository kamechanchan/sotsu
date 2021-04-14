cmake_minimum_required(VERSION 2.8.3)
project(denso_execute)

add_definitions("-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  genmsg
  geometry_msgs
  message_generation
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
  tf
  )

add_message_files(DIRECTORY msg FILES Plan.msg PlanningRequest.msg)
add_action_files(DIRECTORY action FILES ExecutePlan.action)
generate_messages(DEPENDENCIES actionlib_msgs moveit_msgs)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    genmsg
    geometry_msgs
    message_generation
    message_runtime
    moveit_commander
    moveit_core
    moveit_msgs
    moveit_ros_perception
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    roscpp
    std_msgs
    tf
  )


#########
# Build #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(execute_action_server src/execute_action_server.cpp)
add_dependencies(
  execute_action_server
  execute_action_server_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(execute_action_server ${catkin_LIBRARIES})


###########
# Install #
###########

set(EXEC_FILES
  execute_action_server
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()

# Python scripts
set(PYTHON_SCRIPTS
  scripts/parallel_planner.py
  scripts/parallel_executor.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./launch/
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

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()

