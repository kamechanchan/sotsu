cmake_minimum_required(VERSION 2.8.3)
project(denso_gazebo_grasp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  denso_gazebo_msgs
  denso_gazebo_srvs
  geometry_msgs
  moveit_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  std_msgs
  std_srvs
  tercero_gazebo_srvs
  tf2_ros
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    denso_gazebo_msgs
    denso_gazebo_srvs
    roscpp
    tercero_gazebo_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tercero_grasp_service_server
  src/process_vector.cpp
  src/tercero_grasp.cpp
  src/tercero_grasp_server.cpp
)
add_dependencies(tercero_grasp_service_server
  denso_gazebo_srvs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(tercero_grasp_service_server
  ${catkin_LIBRARIES}
)
add_dependencies(tercero_grasp_service_server
  ${PROJECT_NAME}_gencpp)

add_executable(move_and_grasp
  src/move_and_grasp.cpp
)
add_dependencies(move_and_grasp
  denso_gazebo_srvs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_and_grasp
  ${catkin_LIBRARIES}
)

add_executable(grasp_point_tf_publisher
  src/grasp_point_tf_publisher.cpp
)

target_link_libraries(grasp_point_tf_publisher
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  tercero_grasp_service_server
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(DIRECTORIES
  ./launch/
  )

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
