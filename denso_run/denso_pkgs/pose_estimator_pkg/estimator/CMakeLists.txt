cmake_minimum_required(VERSION 2.8.3)
project(estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  color_cloud_bridge
  message_generation
  geometry_msgs
  std_msgs
  cv_bridge
  tf2_ros
)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

add_service_files(
  FILES
  input_data.srv
  bounding.srv
  first_input.srv
  second_input.srv
  nearest_mesh.srv
  third_input.srv
  fourth_input.srv
  five_input.srv
  six_input.srv
  tf_quat.srv
)

# add_message_files(
#   FILES
#   2D_to_3D.msg
#   uv_points.msg
#   matryoshka.msg
#   relay.msg
# )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  # uv_points
  # relay
  # matryoshka
  # std_msgs
  # cv::Mat
  #   tf2_geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(color_pointcloud src/color_pointcloud.cpp)
target_link_libraries(color_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(pcl_pub src/pcl_pub.cpp)
target_link_libraries(pcl_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

add_executable(server_2D src/server_2D.cpp)
target_link_libraries(server_2D
  ${catkin_LIBRARIES}
  # ${OpenCV_LIBRARIES}
)

add_executable(ishiyama_img_naze src/ishiyama_img_naze.cpp)
target_link_libraries(ishiyama_img_naze
  ${catkin_LIBRARIES}
  # ${OpenCV_LIBRARIES}
)

add_executable(2D_to_3D src/2D_to_3D.cpp)
add_dependencies(2D_to_3D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(2D_to_3D
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBRARIES}
)

add_executable(2D_to_3D_insnum src/2D_to_3D_insnum.cpp)
add_dependencies(2D_to_3D_insnum ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(2D_to_3D_insnum
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OpenCV_LIBRARIES}
)
###########
# Install #
###########


set(DIRECTORIES
  launch)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all files
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
