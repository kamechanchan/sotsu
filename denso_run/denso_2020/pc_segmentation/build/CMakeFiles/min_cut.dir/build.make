# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build

# Include any dependencies generated for this target.
include CMakeFiles/min_cut.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/min_cut.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/min_cut.dir/flags.make

CMakeFiles/min_cut.dir/src/min_cut.cpp.o: CMakeFiles/min_cut.dir/flags.make
CMakeFiles/min_cut.dir/src/min_cut.cpp.o: ../src/min_cut.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/min_cut.dir/src/min_cut.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/min_cut.dir/src/min_cut.cpp.o -c /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/src/min_cut.cpp

CMakeFiles/min_cut.dir/src/min_cut.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/min_cut.dir/src/min_cut.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/src/min_cut.cpp > CMakeFiles/min_cut.dir/src/min_cut.cpp.i

CMakeFiles/min_cut.dir/src/min_cut.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/min_cut.dir/src/min_cut.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/src/min_cut.cpp -o CMakeFiles/min_cut.dir/src/min_cut.cpp.s

# Object files for target min_cut
min_cut_OBJECTS = \
"CMakeFiles/min_cut.dir/src/min_cut.cpp.o"

# External object files for target min_cut
min_cut_EXTERNAL_OBJECTS =

min_cut: CMakeFiles/min_cut.dir/src/min_cut.cpp.o
min_cut: CMakeFiles/min_cut.dir/build.make
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_people.so
min_cut: /usr/lib/x86_64-linux-gnu/libboost_system.so
min_cut: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
min_cut: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
min_cut: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
min_cut: /usr/lib/x86_64-linux-gnu/libboost_regex.so
min_cut: /usr/lib/x86_64-linux-gnu/libqhull.so
min_cut: /usr/lib/libOpenNI.so
min_cut: /usr/lib/libOpenNI2.so
min_cut: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libfreetype.so
min_cut: /usr/lib/x86_64-linux-gnu/libz.so
min_cut: /usr/lib/x86_64-linux-gnu/libjpeg.so
min_cut: /usr/lib/x86_64-linux-gnu/libpng.so
min_cut: /usr/lib/x86_64-linux-gnu/libtiff.so
min_cut: /usr/lib/x86_64-linux-gnu/libexpat.so
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_features.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_search.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_io.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
min_cut: /usr/lib/x86_64-linux-gnu/libpcl_common.so
min_cut: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libfreetype.so
min_cut: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
min_cut: /usr/lib/x86_64-linux-gnu/libz.so
min_cut: /usr/lib/x86_64-linux-gnu/libGLEW.so
min_cut: /usr/lib/x86_64-linux-gnu/libSM.so
min_cut: /usr/lib/x86_64-linux-gnu/libICE.so
min_cut: /usr/lib/x86_64-linux-gnu/libX11.so
min_cut: /usr/lib/x86_64-linux-gnu/libXext.so
min_cut: /usr/lib/x86_64-linux-gnu/libXt.so
min_cut: CMakeFiles/min_cut.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable min_cut"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/min_cut.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/min_cut.dir/build: min_cut

.PHONY : CMakeFiles/min_cut.dir/build

CMakeFiles/min_cut.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/min_cut.dir/cmake_clean.cmake
.PHONY : CMakeFiles/min_cut.dir/clean

CMakeFiles/min_cut.dir/depend:
	cd /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build /home/ericlab/ros_package/denso_ws/src/denso_run/denso_2020/pc_segmentation/build/CMakeFiles/min_cut.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/min_cut.dir/depend

