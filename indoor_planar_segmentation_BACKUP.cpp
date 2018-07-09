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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include "planeSegmentation.h"


// global
pcl::visualization::PCLVisualizer viewer ("Segmention Visualization");
pcl::PCDWriter writer;
int input_idx =0;
int inputInt = 30; //default
int v1 = 1;
int v2 = 2;
double input_d = 0.5;
double input_d2 = 2.0;
double ransacDist = 0.02;


// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-r:  ransac distance." << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}


// segment to multiple planes then visualizer
int planarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
 
  // var and init
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PCDWriter writer;
  std::cout << " - Running segmentation function with function -\n" << std::endl;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (ransacDist);
  seg.setOptimizeCoefficients (true);    // Optional

  // Create the filtering to remove sparse points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (source_cloud);
  sor.setMeanK (inputInt);
  sor.setStddevMulThresh (input_d2);
  sor.filter (*cloud_filtered);

  // Create Cluster init
  // Creating the KdTree object for the search method of the extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); // 10cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (25000);

  // normal
  pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals (new pcl::PointCloud<pcl::Normal>);
  planes_avgNormals->width = 10; // size!!!!! number of planes in model
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;

  int cluster_index = 0;
  int thresh = cloud_filtered->points.size()*0.1;
  // =========loop till inliers left with 10% (thresh) left out================
  std::cerr << "Start Loop with total points " << cloud_filtered->points.size() << "   Point Thresh: " << thresh << endl;

  for (size_t idx = 0; cloud_filtered->points.size() > thresh; idx++ ){
    // segmentation
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    std::cerr << "\nIn Loop ||" + std::to_string(idx) << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    // extract from indics
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.filter (*planar_cloud);

    // clusterize each plane
    tree->setInputCloud (planar_cloud); //TODO check if theres any use of kd tree func
    ec.setSearchMethod (tree);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud (planar_cloud);
    ec.extract (cluster_indices);

    std::cout << "Cluster Indics sizes: " << cluster_indices.size () << " data points." << std::endl;


    // extract and visualize cluster segmentation for each plane
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (planar_cloud->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //output
      std::cout << "\n- PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      // add point cloud to visualizer scene
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color_handler (cloud_cluster, j*73 + idx*82, idx*38 - j*22 , 255 - idx*53);
      viewer.addPointCloud (cloud_cluster, cluster_color_handler, std::to_string(idx) + "cloud_cluster" + std::to_string(j), v2);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, std::to_string(idx)+ "cloud_cluster" + std::to_string(j));

      // =================== get normall!!!!!!!!!!!!!!!!! =============
      // get plane normal then get state
      std::string state = getPlaneState( cloud_cluster, planes_avgNormals, cluster_index );
      std::cout << " [cluster " << cluster_index << " ] " << state << std::endl;
      
      viewer.addText3D(state + "_" + std::to_string(cluster_index), cloud_cluster->points[ cloud_cluster->points.size()/2 ], 
        0.12, 0.0, 1.0, 0.0, "cluster_text"+ std::to_string(cluster_index));

      //temp to prevent breakdown
      if (cluster_index== 9) break;

      cluster_index++;
      j++;
    }

    // Extract non-plane returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract.setNegative (false);

    // add point cloud to visualizer scene
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> planar_cloud_color_handler (planar_cloud, idx*82, idx*38 , 255 - idx*53);
    
      //update viewer
    viewer.addPointCloud (planar_cloud, planar_cloud_color_handler, "planar_cloud" + std::to_string(idx), v1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "planar_cloud" + std::to_string(idx));
    viewer.addText3D("PC " + std::to_string(idx), planar_cloud->points[ inliers->indices.size()/2 ], 0.05, 1.0, 0.0, 0.0, "pc_text"+ std::to_string(idx));
    
    // write to fil
    // if (idx == 3){
    //   writer.write<pcl::PointXYZ> ("output_plane_floor.pcd", *planar_cloud, false);
    // }
    // writer.write<pcl::PointXYZ> ("ouput_plane.pcd", *planar_cloud, false);
    // planar cloud 0:
  }

  return 0;
} 


int getVarfromArg(int argc, char** argv, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud ){

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
  if (pcl::console::find_switch (argc, argv, "-i")){
    input_idx = pcl::console::find_argument (argc, argv, "-i") + 1;
    inputInt = std::atoi(argv[input_idx]);
  } 
  std::cout << "Input Int arg for '-i' is " << inputInt << std::endl;


  // find double agument in command line
  if (pcl::console::find_switch (argc, argv, "-d")){
    input_idx = pcl::console::find_argument (argc, argv, "-d") + 1;
    std::stringstream ss( argv[input_idx] );
    if ( !(ss >> input_d))
      std::cout << "Invalid double...\n";
  } 
  std::cout << "Input Double arg for '-d' is " << input_d << std::endl;


    // find double agument in command line
  if (pcl::console::find_switch (argc, argv, "-x")){
    input_idx = pcl::console::find_argument (argc, argv, "-x") + 1;
    std::stringstream ss( argv[input_idx] );
    if ( !(ss >> input_d2))
      std::cout << "Invalid double...\n";
  } 
  std::cout << "Input Double arg for '-x' is " << input_d2 << std::endl;


    // find ransac dist double agument in command line
  if (pcl::console::find_switch (argc, argv, "-r")){
    input_idx = pcl::console::find_argument (argc, argv, "-r") + 1;
    std::stringstream ss( argv[input_idx] );
    if ( !(ss >> ransacDist))
      std::cout << "Invalid double...\n";
  } 
  std::cout << "Input Double arg for '-r' is " << ransacDist << std::endl;
  
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

  std::cout << " ----------end arg--------- \n" << std::endl;
  return 0;
}


// This is the main function
int main (int argc, char** argv){

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  getVarfromArg(argc, argv, source_cloud);

  std::cout<< "Reading " << argv[0] << " with points: " << source_cloud->points.size() << std::endl;

  // viewer init
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // viewer 1
  viewer.addCoordinateSystem (1.0, "v1_axtransformPointCloudis", v1);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, v1); 
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); //viewer 2
  viewer.addCoordinateSystem (1.0, "v2_axis", v2);
  viewer.setBackgroundColor(0.1, 0.1, 0.1, v2); 

  // Transformation from lidar frame to robot frame (juz rotation)
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  float theta_z = M_PI/4;
  float theta_x = M_PI;
  transform.rotate (Eigen::AngleAxisf (theta_z, Eigen::Vector3f::UnitZ()));
  transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));
  std::cout << "Transformation matrix:" << std::endl
            << transform.matrix() <<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

  // ==== Visualization =====
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 255, 255, 255);
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
  viewer.addText("Poincloud Plane Segmentation", 100, 10, "text1");

  // ===== Planar Segmentation =====
  planarSegmentation(transformed_cloud);

  viewer.setPosition(800, 400); // Setting visualiser window position
  viewer.setCameraPosition(-5,-5,5,0,0,1);

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}