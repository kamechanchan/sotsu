cmake_minimum_required(VERSION 2.8.3)
project(image_processing_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  Contrast.srv
  Gamma.srv
  Histogram.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)