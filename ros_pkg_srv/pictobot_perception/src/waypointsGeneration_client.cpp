#include "ros/ros.h"
#include "pictobot_perception/WaypointsGeneration.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <cstdlib>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypointsGeneration_client");
  if (argc != 2)
  {
    ROS_INFO("usage: waypointsGeneration string::pcd_path");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pictobot_perception::WaypointsGeneration>("waypointsGeneration_service");
  pictobot_perception::WaypointsGeneration srv;
  srv.request.pcd_path = argv[1];

  std::cout << "Input Path: " << argv[1] << std::endl;

  if (client.call(srv))
  {
    // ROS_INFO("From Server: %s", srv.response.pose_array.c_str() );
    std::cout << "From server: " << srv.response.pose_array << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
