cmake_minimum_required(VERSION 2.8.3)
project(tercero_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  )

add_service_files(
  FILES
  GetHandReady.srv
  GetHandState.srv
  GetJointPosition.srv
  GetTorque.srv
  Move.srv
  SetTorque.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
