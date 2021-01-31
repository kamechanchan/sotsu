cmake_minimum_required(VERSION 2.8.3)
project(third_2019)

find_package(catkin REQUIRED COMPONENTS
  denso_state_msgs
  message_generation
)

add_service_files(
  FILES
  GetFixedTarget.srv
  WorkPointTFBroadcast.srv
)

generate_messages(
  DEPENDENCIES
  denso_state_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  denso_state_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
# set(PYTHON_SCRIPTS
# )

# foreach(scripts ${PYTHON_SCRIPTS})
#   catkin_install_python(PROGRAMS ${scripts}
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# endforeach()

set(DIRECTORIES
  ./launch/
  )
# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

########
# Test #
########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
