cmake_minimum_required(VERSION 3.5.0)

project(eip_pkg)

message("-- CMAKE_SYSTEM_INFO_FILE: ${CMAKE_SYSTEM_INFO_FILE}")
message("-- CMAKE_SYSTEM_NAME:      ${CMAKE_SYSTEM_NAME}")
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SYSTEM:           ${CMAKE_SYSTEM}")
message("-- CMAKE_SYSTEM_VERSION:   ${CMAKE_SYSTEM_VERSION}")

find_package(Boost  REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
  CATKIN_DEPENDS roscpp std_msgs
  )

###########
## Build ##
###########

file(GLOB ${PROJECT_NAME}_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /usr/src/linux-headers-${CMAKE_SYSTEM_VERSION}/include
  )

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${Boost_LIBRARY_DIRS}
  )
#[[
add_executable(eip_start
  ${CMAKE_CURRENT_SOURCE_DIR}/src/eip_start.cpp
  ${${PROJECT_NAME}_SOURCES}
  )

target_link_libraries(eip_start
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
  idlLinux
  idl115DriverLinux
  )

#############
## Install ##
#############

install(TARGETS eip_start
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

set(DIRECTORIES
  ./launch/
)

foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

set(LIBDIRECTORIES
  ./lib/
)

foreach(dirs ${LIBDIRECTORIES})
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
]]