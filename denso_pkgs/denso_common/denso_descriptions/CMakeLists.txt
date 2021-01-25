cmake_minimum_required(VERSION 2.8.3)
project(denso_descriptions)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

set(DIRECTORIES
  ./cobotta_description/
  ./common_description/
  ./config/
  ./kinectv2_description/
  ./object_description/
  ./phoxi_camera_description/
  ./vs087_description/
  )
# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

set(DIRECTORIES
  vs087
  vs087_and_kinectv2
  vs087_and_photoneo
  vs087_with_iai_gripper
  vs087_with_mhand
  vs087_with_mhand_and_kinectv2
  vs087_with_mhand2
  vs087_with_robotiq
  vs087_with_tercero
  vs087_with_tercero_and_photoneo
  )

set(COBOTTA_DIRECTORIES
  cobotta
  cobotta_with_cobotta_gripper
  )


if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  foreach(dirs ${DIRECTORIES})
    roslaunch_add_file_check(vs087_description/${dirs}/launch)
  endforeach()
  foreach(dirs ${COBOTTA_DIRECTORIES})
    roslaunch_add_file_check(cobotta_description/${dirs}/launch)
  endforeach()
endif()
