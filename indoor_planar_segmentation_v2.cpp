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
#include <fstream>
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
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include "getPlaneNormalState.h"
#include <json/json.h>


// global
pcl::visualization::PCLVisualizer viewer ("Segmention Visualization");
pcl::PCDWriter writer;
int input_idx =0;
int inputInt = 0; //default
int v1 = 1;
int v2 = 2;
double input_d = 0.5;
double input_d2 = 2.0;
Json::Value configParam; // from Json loader


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
int planarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int patchNum){
 
  // var and init
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PCDWriter writer;
  std::cout << " - Running segmentation function for patch " << patchNum << " -" << std::endl;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (configParam["PlanarSegmentation"]["ransacDist"].asDouble());
  seg.setOptimizeCoefficients (true);    // Optional

  // normal
  pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals (new pcl::PointCloud<pcl::Normal>);
  planes_avgNormals->width = 10; // size!!!!! number of planes in model
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;

  double threshpercentage = configParam["PlanarSegmentation"]["cloudFilteredTresh"].asDouble();
  int thresh = cloud_filtered->points.size()*threshpercentage ;
  // =========loop till inliers left with 10% (thresh) left out================
  std::cerr << " - Point Thresh: " << thresh << endl;

  for (size_t idx = 0; cloud_filtered->points.size() > thresh; idx++ ){
    // segmentation
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    std::cerr << "Loop "<< idx  << " | Filtered cloud size: " << cloud_filtered->points.size() << std::endl;

    // extract from indics
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.filter (*planar_cloud);

    // clusterize each plane (refer to old code)

    // Extract non-plane returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract.setNegative (false);
    
    
    // ======== OUTPUT TO VIEWER ==========
    // add point cloud to visualizer scene
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color_handler (planar_cloud, 
      180*(patchNum+1) - idx*30, idx*88 - 22 , 205*patchNum - (idx + 1)*53); //plane colour
    viewer.addPointCloud (planar_cloud, cluster_color_handler, std::to_string(patchNum) + "planar_cloud" + std::to_string(idx), v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, std::to_string(patchNum)+ "planar_cloud" + std::to_string(idx));

    // =================== get normall!!!!!!!!!!!!!!!!! =============
    // get plane normal then get state
    std::string state = getPlaneState( planar_cloud, planes_avgNormals, idx );    
    viewer.addText3D( state + std::to_string(patchNum) + "_" + std::to_string(idx), planar_cloud->points[ planar_cloud->points.size()/2 ], 
      0.12, 0.0, 1.0, 0.0, std::to_string(patchNum)+ "cluster_text"+ std::to_string(idx));
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


// set boundary height of-axis of ceilign and floor according to threshold in config
class BoundaryHeight{

  private:
    double ceiling, floor;

  public:
    BoundaryHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
    
      int index;
      double floor_thresh = configParam["Boundary"]["z_Thresh"][0].asDouble();
      double ceiling_thresh = configParam["Boundary"]["z_Thresh"][1].asDouble();
      std::list<double> z_data;
      std::list<double>::iterator it = z_data.begin();

      for (size_t i=0; i < source_cloud->points.size (); ++i){
        z_data.push_back( source_cloud->points[i].z );
      }
      z_data.sort(); //sort number
      
      index = z_data.size() * ceiling_thresh;
      std::advance(it, index);
      ceiling = *it;
      it = z_data.begin();

      index = z_data.size() * floor_thresh;
      std::advance(it, index);
      floor = *it;
    }

    double Ceiling() { return ceiling; }
    double Floor() { return floor; }
};


class PC2Patches{

  private:
    double floorHeight, ceilingHeight, patchHeight;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ;
    

  public:
      // Set Point cloud boundary according to config param
    pcl::PointCloud<pcl::PointXYZ>::Ptr setPCBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){ // filtered_cloud){
      // build the condition
      cloud_filtered = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;

      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
      BoundaryHeight boundaryHeight(source_cloud);

      // x-axis
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, configParam["Boundary"]["x"][0].asDouble()  )));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, configParam["Boundary"]["x"][1].asDouble()  )));

      // y-axis
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, configParam["Boundary"]["y"][0].asDouble()  )));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, configParam["Boundary"]["y"][1].asDouble()  )));

      // z-axis
      floorHeight = boundaryHeight.Floor();
      ceilingHeight = boundaryHeight.Ceiling();
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, floorHeight  )));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, ceilingHeight  )));

      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
      condrem.setCondition (range_cond);
      condrem.setInputCloud (source_cloud);
      condrem.setKeepOrganized(true);
      
      // apply filter
      condrem.filter (*cloud_filtered);
      return cloud_filtered;
    } 

    //split filtered point cloud to several horizonal patches
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPatch(int patchNum){
      double higher_bound, lower_bound;
      pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

      higher_bound = ceilingHeight - patchHeight*patchNum;
      lower_bound = ceilingHeight - patchHeight*(patchNum + 1);
      if (higher_bound < floorHeight){
        std::cerr << "Error! Patching for " << patchNum << " out of bound!!!" << std::endl;
        return patch_cloud;
      }
      if (lower_bound < floorHeight) { lower_bound = floorHeight; }

      // z-axis filtering
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, lower_bound  )));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
        (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, higher_bound  )));

      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
      condrem.setCondition (range_cond);
      condrem.setInputCloud (cloud_filtered);
      condrem.setKeepOrganized(true);
      condrem.filter (*patch_cloud);  // apply filter

      //remove nan in pointcloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud( *patch_cloud, *patch_cloud, indices );

      std::cout << "PatchNum " << patchNum << " >> " << lower_bound  << "\t" << higher_bound << std::endl; 
      return patch_cloud;
    }

    //get number of patches in filtered point cloud
    int getNumberOfPatches(){
      double totalHeight = ceilingHeight - floorHeight;
      patchHeight = configParam["Patch"]["height"].asDouble();
      std::cout << "Floor to Ceiling: " << totalHeight << std::endl;
      return ceil(totalHeight/patchHeight);
    }

};





// This is the main function
int main (int argc, char** argv){

  // ==== Load files | Works with PCD and PLY files, and Config File ===
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  getVarfromArg(argc, argv, source_cloud);
  std::ifstream config_doc("../config/param.json", std::ifstream::binary); //load json config file
  config_doc >> configParam;
  std::cout << "Config file: param.json is loaded " << std::endl;
  std::cout << "Reading " << argv[0] << " with points: " << source_cloud->points.size() << std::endl;


  // === Transformation from lidar frame to robot frame (juz rotation) ===
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  float theta_z = M_PI/4;
  float theta_x = M_PI;
  transform.rotate (Eigen::AngleAxisf (theta_z, Eigen::Vector3f::UnitZ()));
  transform.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));
  std::cout << "Transformation matrix:" << std::endl
            << transform.matrix() <<std::endl << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);


  // ==== Visualization =====
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // viewer 1
  viewer.addCoordinateSystem (1.0, "v1_transformPointCloud", v1);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, v1); 
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); //viewer 2
  viewer.addCoordinateSystem (1.0, "v2_axis", v2);
  viewer.setBackgroundColor(0.2, 0.2, 0.2, v2);
  // add source cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 255, 255, 255);
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
  viewer.addText("Poincloud Plane Segmentation", 100, 10, "text1");

  
  // ==== Filtering ====
  PC2Patches pc2Patches;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  filtered_cloud = pc2Patches.setPCBoundary(transformed_cloud);//, filtered_cloud);
  // Create the filtering to remove sparse points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (filtered_cloud);
  sor.setMeanK ( configParam["StatisticalOutlierRemoval"]["MeanK"].asInt() );
  sor.setStddevMulThresh ( configParam["StatisticalOutlierRemoval"]["StdDevThresh"].asDouble() );
  sor.filter (*filtered_cloud);
  

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_color_handler (filtered_cloud, 223, 23 , 223);
  viewer.addPointCloud (filtered_cloud, filtered_color_handler, "filtered source cloud", v2);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered source cloud");


  // === Spliting of Patches ===
  int numberOfPatches = pc2Patches.getNumberOfPatches();
  std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudPatches ( numberOfPatches );
  pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  std::cout <<  "number of patches "  << numberOfPatches << std::endl;
  
  //loop thru patches
  for (size_t i= 0 ; i< numberOfPatches ; i++){

    cloudPatches[i] = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
    cloudPatches[i] = pc2Patches.getPatch(i);
    std::cout << "cloud patches size: "<< cloudPatches[i]->points.size () << std::endl;



    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> patch_color_handler (cloudPatches[i], 23 + i*80, 200 , 223- i*70);
    viewer.addPointCloud (cloudPatches[i], patch_color_handler, "filtered_patch_" + std::to_string(i), v1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered_patch_" + std::to_string(i));
    writer.write<pcl::PointXYZ> (std::to_string(i) + "output_patch.pcd", *cloudPatches[i], false);
  }

  // ===== Planar Segmentation =====
  //loop thru the patches for to segment each patch (will multitask nxt time)
  for (size_t i= 0 ; i< numberOfPatches ; i++){
    std::cout << "\n From Main:: Cloud Patch size: "<< cloudPatches[i]->points.size () << std::endl;
    planarSegmentation( cloudPatches[i], i );
  }


  viewer.setPosition(800, 400); // Setting visualiser window position
  viewer.setCameraPosition(-5,-5,5,0,0,1);

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}