cmake_minimum_required(VERSION 2.8.3)
project(second_2019)

find_package(catkin REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

###########
# Install #
###########

# Python scripts
set(PYTHON_SCRIPTS
  scripts/pick_and_place_demo.py
  scripts/target_value_generator.py
  scripts/work_points_tf_broadcaster.py
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
