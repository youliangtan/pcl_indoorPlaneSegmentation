#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <cstdlib>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pictobot_perception/PathTrace.h>
#include <pictobot_perception/PathTraceArray.h>
#include "pictobot_perception/WaypointsGeneration.h"


// load input pcd file
void loadPCD(std::string file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
  if (pcl::io::loadPCDFile (file_path, *source_cloud) < 0)  {
    std::cout << "Error loading point cloud " << file_path << std::endl << std::endl;
    exit(0); 
  }
  std::cout << "Reading .pcd file " << file_path << " with points:  " << source_cloud->points.size() << std::endl;
} 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypointsGeneration_client");
  if (argc != 2)
  {
    ROS_INFO("\n Please Note>> usage: waypointsGeneration string::pcd_path");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pictobot_perception::WaypointsGeneration>("waypointsGeneration_service");
  pictobot_perception::WaypointsGeneration srv;
  srv.request.pcd_path = argv[1]; //TODO need remove
  
  //load pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  loadPCD(argv[1], source_cloud);

  //send pointcloud msg format
  sensor_msgs::PointCloud2 point_cloud;
  pcl::toROSMsg(*source_cloud, point_cloud);
  srv.request.point_cloud = point_cloud;

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
