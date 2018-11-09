/*
*Create By Tan You Liang, Under Transforma Project
*Jul 2018, CopyRight
*Acknowledge before using this code
*/


#include "planeSegmentation.h"
#include "patchSegmentation.h"
#include "pointcloudSegmentation.h"


Json::Value configParam; // from Json loader


void readConfigFile(std::string path){
  try {
    std::ifstream config_doc(path, std::ifstream::binary); //load json config file
    config_doc >> configParam;
    std::cout << "== Config file: param.json is loaded ==" << std::endl;
  }
  catch (...) { 
    std::cout << "JSON Exception: Change to backup path "<< std::endl; 
    path = "config/param.json"; //try second path << on ros service
    try {
      std::ifstream config_doc(path, std::ifstream::binary); //load json config file
      config_doc >> configParam;
      std::cout << "== Config file: param.json is loaded ==" << std::endl;
    }
    catch (...) { 
      std::cout << "ERROR ON READING JSON!!! "<< std::endl; 
      std::cout << "Check if Json path is " << path << std::endl; 
      exit(0);
    }
  }
}


// used in main segmentation and plane segmentation before clustering
void outlinerFiltering( pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud ){
  
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (filtered_cloud);
  // sor.setMeanK ( configParam["OutlierRemoval"]["S_MeanK"].asInt() );
  // sor.setStddevMulThresh ( configParam["OutlierRemoval"]["S_StdDevThresh"].asDouble() );
  // sor.filter (*filtered_cloud);

  //remove nan in pointcloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud( *filtered_cloud, *filtered_cloud, indices );

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(filtered_cloud); // build the filter
  outrem.setMinNeighborsInRadius( configParam["OutlierRemoval"]["R_minNeighbours"].asInt() );
  outrem.setRadiusSearch ( configParam["OutlierRemoval"]["R_radius"].asDouble() );
  outrem.filter (*filtered_cloud); 
}


// input pointcloud
// function will segment and visualize the segmentation
int PointCloudSegmentation(  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, std::vector < PlaneStruct > *cloudPlanes, PCVisualizer &seg_viewer ){

  // read config file
  readConfigFile("../config/param.json");


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

  // add source cloud
  // PCVisualizer seg_viewer;
  seg_viewer.addSource(transformed_cloud);
  
  // ==== Filtering ====
  PC2Patches pc2Patches;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  filtered_cloud = pc2Patches.setPCBoundary(transformed_cloud);//, filtered_cloud);
  outlinerFiltering( filtered_cloud );

  // === Spliting of Patches ===
  int numberOfPatches = pc2Patches.getNumberOfPatches();
  std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudPatches ( numberOfPatches );
  pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  std::cout <<  "number of patches "  << numberOfPatches << std::endl;

  //loop thru patches
  for (size_t i= 0 ; i< numberOfPatches ; i++){
    cloudPatches[i] = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
    cloudPatches[i] = pc2Patches.getPatch(i);
    // visualizer on viewer 1
    seg_viewer.addPatch (cloudPatches[i], i);
  }

  // ===== Planar Segmentation =====
  PlaneSeg patch_seg;
  //loop thru the patches for to segment each patch (will multitask nxt time)
  for (size_t i= 0 ; i< numberOfPatches ; i++){
    std::cout << "\n From Main:: Cloud Patch size: "<< cloudPatches[i]->points.size () << std::endl;
    patch_seg.patchPlanarSegmentation( cloudPatches[i], cloudPlanes, i);
  }
  // get exception planes
  std::cout << "\n - Running Exception plane segmentation -" << std::endl;
  for (size_t i= 1 ; i< numberOfPatches ; i++){
    if (patch_seg.exceptionPlanarSegmentation( pc2Patches.getExceptionPatch(i), cloudPlanes, i ) == 1){ 
        std::cout << "exception plane is added with type: " << (*cloudPlanes)[ cloudPlanes->size() - 1 ].type << std::endl;
    };
  }

  // output to visualizer or writer
  seg_viewer.addPlaneStruct( cloudPlanes );
  // seg_viewer.runViewer();


  std::cout << "cloud size " << cloudPlanes->size() << std::endl;

  return numberOfPatches;
}
