/*
General Sequential Methodolody:
1) input source pcd file
2) Mophological Filtering
3) Planar detection/ Segmentation
4) Classification of planes (ceilings, floor, wall, curbs)
5) Visualize Point Cloud
6) Grouping of point clouds
*/

#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>


// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}


// This is the main function
int
main (int argc, char** argv)
{

  // ============ Init ===================

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");


  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  // find int agument in command line
  int inputInt = 30; //default
  if (pcl::console::find_switch (argc, argv, "-i")){
    int input_idx = pcl::console::find_argument (argc, argv, "-i") + 1;
    inputInt = std::atoi(argv[input_idx]);
    std::cout << "Input Int arg for '-i' is " << inputInt << std::endl;
  } 

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }

  std::cout<< "Reading " << argv[0] << " with points: " << source_cloud->points.size() << std::endl;


  // ===== Planar Segmentation =====
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);
  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setInputCloud (source_cloud);
  seg.segment (*inliers, *coefficients);

  // if (inliers->indices.size () == 0)
  // {
  //   PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  //   return (-1);
  // }

  // for (size_t i = 0; i < inliers->indices.size (); ++i){
  //   std::cerr << inliers->indices[i] << "    " << source_cloud->points[inliers->indices[i]].x << " "
  //                                              << source_cloud->points[inliers->indices[i]].y << " "
  //                                              << source_cloud->points[inliers->indices[i]].z << "   idx:" << inliers->indices[i] <<std::endl;
  // }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PCDWriter writer;

  pcl::visualization::PCLVisualizer viewer ("Segmention Visualization");

  // Create the filtering to remove sparse points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (source_cloud);
  sor.setMeanK (inputInt);
  sor.setStddevMulThresh (2.0);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("SOR_filtered.pcd", *cloud_filtered, false);

  //loop till inliers left with 15% (thresh) left out
  int thresh = source_cloud->points.size()*0.05;
  std::cerr << "Start Loop with inlinears: " << inliers->indices.size() << "   Point Thresh: " << thresh << endl;

  for (size_t idx = 0; inliers->indices.size() > thresh; idx++ ){
    // segmentation
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    std::cerr << "In Loop ||" + std::to_string(idx) << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    // extract
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.filter (*planar_cloud);

    // Extract non-plane returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract.setNegative (false);

    // add point cloud to visualizer scene
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> planar_cloud_color_handler (planar_cloud, idx*60, 50 + idx*40, 255 - idx*60);
    viewer.addPointCloud (planar_cloud, planar_cloud_color_handler, "planar_cloud" + std::to_string(idx));
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planar_cloud" + std::to_string(idx));

    viewer.addText3D("PC " + std::to_string(idx), planar_cloud->points[ inliers->indices.size()/2 ], 0.05, 1.0, 0.0, 0.0);
    // write to fil  // //viewer.setPosition(800, 400); // Setting visualiser window position
    // writer.write<pcl::PointXYZ> ("ouput_plane.pcd", *planar_cloud, false);
  }

  // ==== Visualization =====
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "source_cloud");

  // Final settings
  viewer.addCoordinateSystem (1.0, "cloud_", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");

  viewer.addText("Poincloud Plane Segmentation", 100, 10, "text1");

  // //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }



  return 0;
}