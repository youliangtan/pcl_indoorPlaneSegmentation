#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


int
 main (int argc, char** argv)
{
  // ini var for pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // plane_cloud->width    = 15;//inliers->indices.size () + 5;
  // plane_cloud->height   = 1;
  // plane_cloud->points.resize (plane_cloud->width * plane_cloud->height);


  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << "   idx:" << i << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  // set new size for planar point cloud
  std::cerr << "Num of Planar Points " << inliers->indices.size () << std::endl;
  plane_cloud->width    = inliers->indices.size ();
  plane_cloud->height   = 1;
  plane_cloud->points.resize (plane_cloud->width * plane_cloud->height);

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i){
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << "   idx:" << inliers->indices[i] <<std::endl;
  
    // create new pointcloud for plane
    plane_cloud->points[i].x = cloud->points[inliers->indices[i]].x;
    plane_cloud->points[i].y = cloud->points[inliers->indices[i]].y;
    plane_cloud->points[i].z = cloud->points[inliers->indices[i]].z;
  }

  // write to file
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("ouput.pcd", *cloud, false);
  writer.write<pcl::PointXYZ> ("ouput_plane.pcd", *plane_cloud, false);

  // // Add-on: ==== Visualization =====

  // printf(  "\nPoint cloud colors :  white  = original point cloud\n"
  //     "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Segmention Visualization");

  //  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
  // // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_cloud_color_handler (plane_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (plane_cloud, plane_cloud_color_handler, "plane_cloud");

  viewer.addCoordinateSystem (1.0, "cloud_", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "plane_cloud");
  // //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }



  return (0);
}
