cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(supervoxel_clustering)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (supervoxel_clustering supervoxel_clustering.cpp)
add_executable (cluster_extraction cluster_extraction.cpp)
add_executable (normalEstimation normalEstimation.cpp)

target_link_libraries (supervoxel_clustering ${PCL_LIBRARIES}) 
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
target_link_libraries (normalEstimation ${PCL_LIBRARIES})

