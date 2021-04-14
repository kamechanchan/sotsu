cmake_minimum_required(VERSION 2.8.3)
project(denso_calib_gazebo)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

# script
file(GLOB prgs RELATIVE ${PROJECT_SOURCE_DIR} "scripts/*.py")
foreach(prg ${prgs})
  catkin_install_python(PROGRAMS ${prg}
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

