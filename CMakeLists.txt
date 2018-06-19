cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(pointCloudProject)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pc_visualizer pc_visualizer.cpp)
add_executable (planar_segmentation planar_segmentation.cpp)
add_executable (indoor_planar_segmentation indoor_planar_segmentation.cpp)
add_executable (concave_hull_2d concave_hull_2d.cpp)

target_link_libraries (pc_visualizer ${PCL_LIBRARIES})
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
target_link_libraries (indoor_planar_segmentation ${PCL_LIBRARIES})
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

