cmake_minimum_required(VERSION 3.0.2)
project(tf_publish)

# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  rospy
  tf
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_publish
#  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions roscpp rospy tf tf2_ros
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tf_publish.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_executable(${PROJECT_NAME}_node src/tf_publish_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(odom_tf_converter src/odom_tf_converter.cpp)
target_link_libraries(odom_tf_converter
  ${catkin_LIBRARIES}
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(tf_lister src/tf_lister.cpp)
target_link_libraries(tf_lister
  ${catkin_LIBRARIES}
)

add_executable(model_tf_publisher src/model_tf_publisher.cpp src/model_tf_publisher_node.cpp)
target_link_libraries(model_tf_publisher
  ${catkin_LIBRARIES}
)

add_executable(one_message_pub src/one_message_pub.cpp)
target_link_libraries(one_message_pub
  ${catkin_LIBRARIES}
)

add_executable(one_message_sub src/one_message_sub.cpp)
target_link_libraries(one_message_sub
  ${catkin_LIBRARIES}
)

add_executable(model_name src/model_name.cpp)
target_link_libraries(model_name
  ${catkin_LIBRARIES}
)

add_executable(model_tf_publisher_bara src/model_tf_publisher_bara.cpp src/model_tf_publisher_bara_node.cpp)
target_link_libraries(model_tf_publisher_bara
  ${catkin_LIBRARIES}
)