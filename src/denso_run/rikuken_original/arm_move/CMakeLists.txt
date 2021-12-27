cmake_minimum_required(VERSION 3.0.2)
project(arm_move)


# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  actionlib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES arm_move
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


# add_executable(trajectory_move src/trajctory_move.cpp)
# target_link_libraries(trajectory_move
#   ${catkin_LIBRARIES}
# )

# add_executable(trajectory_goal_move src/trajectory_goal_move.cpp)
# target_link_libraries(trajectory_goal_move
#   ${catkin_LIBRARIES}
# )

# add_library(arm_move src/arm_move.cpp)
# target_link_libraries(arm_move
#   ${catkin_LIBRARIES}
# )

# add_executable(arm_move_main_1 src/arm_move_main_1.cpp)
# target_link_libraries(arm_move_main_1
#   ${catkin_LIBRARIES}
#   arm_move
# )

# add_executable(arm_move_main_2 src/arm_move_main_2.cpp)
# target_link_libraries(arm_move_main_2
#   ${catkin_LIBRARIES}
#   arm_move
# )

# add_executable(tf_subscriber src/tf_subscriber.cpp)
# target_link_libraries(tf_subscriber
#   ${catkin_LIBRARIES}
# )
# file(MAKE_DIRECTORY /opt/ros/noetic/include/arm_move)
# file(MAKE_DIRECTORY /opt/ros/noetic/lib/arm_move)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION 
#   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
# )