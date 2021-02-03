cmake_minimum_required(VERSION 2.8.3)
project(image_processing_pkg)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_processing_srvs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)

# test nodes
add_executable(contrast_test
  src/test_node/contrast_test.cpp)
target_link_libraries(contrast_test
  ${catkin_LIBRARIES})
add_executable(gamma_test
  src/test_node/gamma_test.cpp)
target_link_libraries(gamma_test
  ${catkin_LIBRARIES})
add_executable(histogram_test
  src/test_node/histogram_test.cpp)
target_link_libraries(histogram_test
  ${catkin_LIBRARIES})

# main processing modifiers
add_executable(image_processing_server
  src/image_processing_server.cpp
  src/contrast_modifier.cpp
  src/gamma_modifier.cpp
  src/histogram_flattening.cpp)
add_dependencies(image_processing_server
  ${image_processing_server_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_processing_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(contrast_modifier
  src/contrast_modifier_node.cpp
  src/contrast_modifier.cpp)
add_dependencies(contrast_modifier
  ${contrast_modifier_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(contrast_modifier
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
add_executable(gamma_modifier
  src/gamma_modifier_node.cpp
  src/gamma_modifier.cpp)
add_dependencies(gamma_modifier
  ${gamma_modifier_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(gamma_modifier
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(histogram_flattening
  src/histogram_flattening_node.cpp
  src/histogram_flattening.cpp)
add_dependencies(histogram_flattening
  ${histogram_flattening_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(histogram_flattening
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

###########
# Install #
###########

set(EXEC_FILES
  image_processing_server
  gamma_modifier
  contrast_modifier
  histogram_flattening
  )
# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(DIRECTORIES
  ./launch/
  )
# Install other files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

set(DIRECTORIES
  ./launch/
  )
foreach(dirs ${DIRECTORIES})
  roslaunch_add_file_check(${dirs})
  endforeach()
endif()
