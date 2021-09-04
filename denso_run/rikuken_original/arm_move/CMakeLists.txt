cmake_minimum_required(VERSION 3.0.2)
project(arm_move)


# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_move
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_move src/trajctory_move.cpp)
target_link_libraries(trajectory_move
  ${catkin_LIBRARIES}
)

add_executable(trajectory_goal_move src/trajectory_goal_move.cpp)
target_link_libraries(trajectory_goal_move
  ${catkin_LIBRARIES}
)