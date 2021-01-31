cmake_minimum_required(VERSION 2.8.3)
project(iai_gripper_srvs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Move.srv
  OriginReturn.srv
  Pause.srv
  Servo.srv
  GetPosition.srv
  Reset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
