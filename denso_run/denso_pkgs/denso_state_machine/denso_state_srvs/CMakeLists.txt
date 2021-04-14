cmake_minimum_required(VERSION 2.8.3)
project(denso_state_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  denso_state_msgs
)

add_service_files(
  FILES
  SetWorkPoint.srv
  GetFixedJointValues.srv
  measurement/Measurement.srv
  measurement/MeasurementWorkPoint.srv
  moving/Moving.srv
  WorkPointServiceProvider.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  denso_state_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
  denso_state_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
