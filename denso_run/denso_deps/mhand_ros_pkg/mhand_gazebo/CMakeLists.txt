cmake_minimum_required(VERSION 2.8.3)
project(mhand_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  )

catkin_package()

#############
## Install ##
#############

set(PYTHON_SCRIPTS
  scripts/driver.py
  scripts/listener.py
  )
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./config/
  ./launch/
  ./robots/
  ./urdf/
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
  roslaunch_add_file_check(launch)
endif()
