cmake_minimum_required(VERSION 2.8.3)
project(teaching_booth)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

set(PYTHON_SCRIPTS
  scripts/tf_interactive_marker.py
  )

foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

set(DIRECTORIES
  ./launch/
  ./rviz/
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

  set(DIRECTORIES
    ./launch/
    )

  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(${dirs})
  endforeach()
endif()
