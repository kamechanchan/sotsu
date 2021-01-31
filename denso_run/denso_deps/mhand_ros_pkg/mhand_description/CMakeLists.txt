cmake_minimum_required(VERSION 2.8.3)
project(mhand_description)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

file(GLOB descs RELATIVE ${PROJECT_SOURCE_DIR} "*_description")
foreach(dir ${descs})
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

set(DIRECTORIES
  ./launch/
  ./meshes/
  ./robots/
  ./urdf/
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
  roslaunch_add_file_check(launch)
endif()
