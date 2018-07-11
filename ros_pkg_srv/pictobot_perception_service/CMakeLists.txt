cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(pictobot_perception_service)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  message_generation  
)

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  WaypointsGeneration.srv   
)

add_message_files(
  FILES
  Num.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PointCloudReceiver
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)

pkg_check_modules(JSONCPP jsoncpp)
include_directories("${JSONCPP_INCLUDE_DIRS}")

link_libraries(${JSONCPP_LIBRARIES})

# include_directories(${JSONCPP_INCLUDE_DIRS})
# link_directories(${JSONCPP_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS}   ${catkin_INCLUDE_DIRS}  src/include)
link_directories(${PCL_LIBRARY_DIRS}  ${catkin_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})


#add_executable (indoor_planar_segmentation_v2 src/indoor_planar_segmentation_v2.cpp src/planeSegmentation.cpp)
#target_link_libraries (indoor_planar_segmentation_v2 ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})


add_executable(waypointsGeneration_server src/waypointsGeneration_server.cpp src/pointcloudSegmentation.cpp src/planeSegmentation.cpp src/patchSegmentation.cpp)
target_link_libraries(waypointsGeneration_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES} )
add_dependencies(waypointsGeneration_server pictobot_perception_service_gencpp)

add_executable(waypointsGeneration_client src/waypointsGeneration_client.cpp  )
target_link_libraries(waypointsGeneration_client ${catkin_LIBRARIES})
add_dependencies(waypointsGeneration_client pictobot_perception_service_gencpp)
