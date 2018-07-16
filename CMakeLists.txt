cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(pointCloudProject)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories("${JSONCPP_INCLUDE_DIRS}")

link_libraries(${JSONCPP_LIBRARIES})

# include_directories(${JSONCPP_INCLUDE_DIRS})
# link_directories(${JSONCPP_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS} src/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pc_visualizer src/pc_visualizer.cpp)
# add_executable (planar_segmentation planar_segmentation.cpp)
add_executable (indoor_planar_segmentation src/indoor_planar_segmentation.cpp src/pointcloudSegmentation.cpp 
src/planeSegmentation.cpp src/patchSegmentation.cpp src/ceilPath.cpp src/wallPath.cpp src/pathIntegrate.cpp)

target_link_libraries (pc_visualizer ${PCL_LIBRARIES})
target_link_libraries (indoor_planar_segmentation ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})