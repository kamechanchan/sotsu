cmake_minimum_required(VERSION 2.8.3)
project(iai_gripper_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  )

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_srvs
  )

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
set(PYTHON_SCRIPTS
  src/activate.py
  src/driver.py
  src/listener.py
  src/monitoring_alarm.py
)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

set(DIRECTORIES
  ./launch/
  )
# Install all files
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
