#include "ros/ros.h"
#include "pictobot_perception_service/WaypointsGeneration.h"
// segmentation header
#include "pointcloudSegmentation.h"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry> 


// global pose array 
geometry_msgs::PoseArray block_poses_;

void addBlock(double x, double y, double z, double angle){
  geometry_msgs::Pose block_pose;
  block_pose.position.x = x;
  block_pose.position.y = y;
  block_pose.position.z = z;

  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(angle, Eigen::Vector3d(0,0,1)));

  block_pose.orientation.x = quat.x();
  block_pose.orientation.y = quat.y();
  block_pose.orientation.z = quat.z();
  block_pose.orientation.w = quat.w();

  //ROS_INFO_STREAM("Added block: \n" << block_pose );
  block_poses_.poses.push_back(block_pose);
}


void createBlock(){
  // creaate pose array block
  block_poses_.header.stamp = ros::Time::now();
  block_poses_.header.frame_id = "base link";

  for (double i=3; i < 5; i += 0.5){
    addBlock(i+1.2, i, i-1.2, i/2);
  }
}


void showHelp(std::string program_name){
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// load input pcd file
void loadPCD(std::string file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
  if (pcl::io::loadPCDFile (file_path, *source_cloud) < 0)  {
    std::cout << "Error loading point cloud " << file_path << std::endl << std::endl;
    showHelp (file_path);
    exit(0); 
  }
  std::cout << "Reading .pcd file " << file_path << " with points:  " << source_cloud->points.size() << std::endl;
}  


// call back for server function
bool service_callback(pictobot_perception_service::WaypointsGeneration::Request  &req,
                        pictobot_perception_service::WaypointsGeneration::Response &res){

  // res.pose_array = req.pcd_path;
  ROS_INFO("request: path=%s", req.pcd_path.c_str() );

  // segmentation portion
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  loadPCD(req.pcd_path, source_cloud);
  main_PointCloudSegmentation( source_cloud );

  // create poseArray Block
  createBlock();
  res.pose_array = block_poses_;

  ROS_INFO("sending back response" );//: %s", res.pose_array.c_str() );
  return true;
}


// main service function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypointsGeneration_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("waypointsGeneration_service", service_callback);
  ROS_INFO("Ready to generate pose array.");
  ros::spin();

  return 0;
}
