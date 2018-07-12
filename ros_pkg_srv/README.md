# pictobot_perception
ROS PKG SERVICE

## Introduction
A waypoint creation package with the input a pointcloud path. This package works as the same as the source file in the same main folder. The only add-on is that this package works as a ROS Service package within ROS. In other words, the same function now will act as a server to the user. 


## Input & Output
Input: PointCloud
> string pcd_path

Output: Array of pose(xyz,rpy)
> geometry_msgs/PoseArray pose_array


## Brief Setup
This ros package consisted of the of multiple folders in within. In order to create a local pakage for the local pc, move the package source folder into a src folder of a constructed catkin_ws. `~/catkin_ws/src` . the CMakeLists.txt will describe all the depencies which will be needed

```
cd ~/catkin_ws
catkin_make -j4
```

The built exe files will be constructed in `~/catkin_ws/devel/lib`. Use rosrun to run the package.

>Run Server: `rosrun pictobot_perception waypointsGeneration_server`

>Run Client: `rosrun pictobot_perception waypointsGeneration_client  'input_pcd_path.pcd'`

A visualizer viewer will be shown by the server, and way points will be returned to client.


** note: the the pakage has the same source file in `.../src` and `.../ros_pkg_srv/picto_perception/src`
