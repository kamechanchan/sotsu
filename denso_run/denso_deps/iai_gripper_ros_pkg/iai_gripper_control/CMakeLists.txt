cmake_minimum_required(VERSION 2.8.3)
project(iai_gripper_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
)

#########
# Build #
#########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(iai_gripper_joint_state_republisher 
  src/iai_gripper_joint_state_republisher.cpp
  src/iai_gripper_joint_state_republisher_node.cpp
)
target_link_libraries(iai_gripper_joint_state_republisher
  ${catkin_LIBRARIES}
)

###########
# Install #
###########

set(EXEC_FILES
  iai_gripper_joint_state_republisher
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS iai_gripper_joint_state_republisher
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
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

########
# Test #
########
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()