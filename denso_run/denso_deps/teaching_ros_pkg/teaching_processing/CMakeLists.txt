cmake_minimum_required(VERSION 2.8.3)
project(teaching_processing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  roslib
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS PCL
)

###########
## build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(load_pcd
  src/load_pcd.cpp
  src/load_pcd_node.cpp
)

target_link_libraries(load_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(save_pcd
  src/save_pcd.cpp
  src/save_pcd_node.cpp
)

target_link_libraries(save_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  load_pcd
  save_pcd
)

# Install executables
foreach(nodes ${EXEC_FILES})
install(TARGETS ${nodes}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(PYTHON_SCRIPTS
  scripts/get_assemble_relative_posture.py
  scripts/hand_detector.py
  scripts/object_tf_broadcaster.py
  scripts/save_tf_pose.py
  )

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./config/
  ./data/
  ./hand/
  ./launch/
  )

# other files
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
