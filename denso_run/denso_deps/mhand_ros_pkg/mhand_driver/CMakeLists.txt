cmake_minimum_required(VERSION 2.8.3)
project(mhand_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
set(PYTHON_SCRIPTS
  src/driver.py
  src/listener.py
  src/set_default_torque.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

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
