cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator_srvs)

find_package(catkin REQUIRED COMPONENTS
  pose_estimator_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_service_files(
  FILES
  PoseEstimate.srv
  range1.srv
)

generate_messages(
  DEPENDENCIES
  pose_estimator_msgs
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  pose_estimator_msgs
  geometry_msgs
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
