cmake_minimum_required(VERSION 3.0)

project(supervoxel_clustering)

add_compile_options(-std=c++17)

find_package(PCL 1.8 REQUIRED)
find_package(GTest REQUIRED)

include_directories(
  include
  ${GTEST_DIR}
  ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (abc_algorithm_run src/abc_algorithm_run.cpp)
target_link_libraries (abc_algorithm_run ${PCL_LIBRARIES})

add_executable (super_voxel src/super_voxel.cpp)
target_link_libraries (super_voxel ${PCL_LIBRARIES})

add_executable (conditional_euclidean src/conditional_euclidean.cpp)
target_link_libraries (conditional_euclidean ${PCL_LIBRARIES})

add_executable (euclidean src/euclidean.cpp)
target_link_libraries (euclidean ${PCL_LIBRARIES})

add_executable (diff_points src/diff_points.cpp)
target_link_libraries (diff_points ${PCL_LIBRARIES})

add_executable (pcd_ascii_save src/pcd_ascii_save.cpp)
target_link_libraries (pcd_ascii_save ${PCL_LIBRARIES})

add_executable (cloud_view src/cloud_view.cpp)
target_link_libraries (cloud_view ${PCL_LIBRARIES})

add_executable (extract_segmentation_pc src/extract_segmentation_pc.cpp)
target_link_libraries (extract_segmentation_pc ${PCL_LIBRARIES})

add_executable (cylinder_seg src/cylinder_seg.cpp)
target_link_libraries (cylinder_seg ${PCL_LIBRARIES})

add_executable (pcd_convert_ascii src/pcd_convert_ascii.cpp)
target_link_libraries (pcd_convert_ascii ${PCL_LIBRARIES})

add_executable (colored_cloud_view src/colored_cloud_view.cpp)
target_link_libraries (colored_cloud_view ${PCL_LIBRARIES})

add_executable (planar_seg src/planar_seg.cpp)
target_link_libraries (planar_seg ${PCL_LIBRARIES})

add_executable (region_growing src/region_growing.cpp)
target_link_libraries (region_growing ${PCL_LIBRARIES})

add_executable (make_hollowed_points src/make_hollowed_points.cpp)
target_link_libraries (make_hollowed_points ${PCL_LIBRARIES})

add_executable (min_cut src/min_cut.cpp)
target_link_libraries (min_cut ${PCL_LIBRARIES})

#############
## Testing ##
#############

add_executable (unit_test_super_voxel test/test_super_voxel.cpp)

add_executable (unit_test_abc_algorithm test/test_abc_algorithm.cpp)

target_link_libraries (unit_test_super_voxel
  ${PCL_LIBRARIES}
  GTest::GTest
  GTest::Main
  )

target_link_libraries (unit_test_abc_algorithm
  ${PCL_LIBRARIES}
  GTest::GTest
  GTest::Main
  )
enable_testing()

add_test(
  NAME test_super_voxel
  COMMAND $<TARGET_FILE:unit_test_super_voxel>
  )

add_test(
  NAME test_abc_algorithm
  COMMAND $<TARGET_FILE:unit_test_abc_algorithm>
  )
