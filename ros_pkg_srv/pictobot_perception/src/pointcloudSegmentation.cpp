
#include "planeSegmentation.h"
#include "patchSegmentation.h"
#include "pointcloudSegmentation.h"


Json::Value configParam; // from Json loader


// PC visualizer
class SegmentationVisualizer{

  pcl::visualization::PCLVisualizer viewer;
  pcl::PCDWriter writer; //init
  int v1, v2;

  public:

    SegmentationVisualizer( ){
      viewer.setWindowName ( "Segmention Visualization" );
      v1 = 1; // viewer port num
      v2 = 2;
      viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // viewer 1
      viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); //viewer 2  
      viewer.addCoordinateSystem (1.0, "v1_transformPointCloud", v1);
      viewer.setBackgroundColor(0.05, 0.05, 0.05, v1); 
      viewer.addCoordinateSystem (1.0, "v2_axis", v2);
      viewer.setBackgroundColor(0.2, 0.2, 0.2, v2);
    }

    // view cloud as white for 2 viewers
    void addSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud ){
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (source_cloud, 255, 255, 255);
      viewer.addPointCloud (source_cloud, cloud_color_handler, "source_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
      viewer.addText("Pointcloud Plane Segmentation", 100, 10, "=')");
    }

    // visualizer single patch in viewer 2
    void addPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPatch, int index ){
      std::cout << "cloud patches size: "<< cloudPatch->points.size () << std::endl;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> patch_color_handler (cloudPatch, 23 + index*80, 200 , 223- index*70);
      viewer.addPointCloud (cloudPatch, patch_color_handler, "filtered_patch_" + std::to_string(index), v1);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "filtered_patch_" + std::to_string(index));
    }

    //output vector of PlaneStruct to viewer 2 
    void addPlaneStruct( std::vector < PlaneStruct > *cloudPlanes ){
      
      std::cout << "\n - Running Visualizer -" << std::endl;
      for (size_t i= 0 ; i< cloudPlanes->size() ; i++ ){

        // add point cloud to visualizer scene
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color_handler ( (*cloudPlanes)[i].cloud, 
          180*(i+1) , i*88 - 22 , (i+1)*53); //plane colour
        viewer.addPointCloud ( (*cloudPlanes)[i].cloud, cluster_color_handler, "planar_cloud" + std::to_string(i), v2);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, 
          "planar_cloud" + std::to_string(i));  
        viewer.addText3D( (*cloudPlanes)[i].type + std::to_string(i) , (*cloudPlanes)[i].cloud->points[ (*cloudPlanes)[i].cloud->points.size() / 2 ], 
          0.12, 0.0, 1.0, 0.0, "cluster_text"+ std::to_string(i));
        std::cout << (*cloudPlanes)[i].type << i << " with plane size: " << (*cloudPlanes)[i].cloud->points.size() << std::endl;
      
        // write to output
        // writer.write<pcl::PointXYZ> ( 
          // "../output/" + (*cloudPlanes)[i].type + "_p" + std::to_string((*cloudPlanes)[i].patchNum) + "_" + std::to_string((*cloudPlanes)[i].planeNum) + ".pcd"
          // , *(*cloudPlanes)[i].cloud, false);
      }
    }

    // run viewer to visualize previous added pointclouds
    void runViewer(){
      viewer.setPosition(800, 400); // Setting visualiser window position
      viewer.setCameraPosition(-5,-5,5,0,0,1);
      while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
      }
    }
};


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

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(filtered_cloud); // build the filter
  outrem.setMinNeighborsInRadius( configParam["OutlierRemoval"]["R_minNeighbours"].asInt() );
  outrem.setRadiusSearch ( configParam["OutlierRemoval"]["R_radius"].asDouble() );
  outrem.filter (*filtered_cloud); 
}


// input pointcloud
// function will segment and visualize the segmentation
int main_PointCloudSegmentation(  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud ){

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
  SegmentationVisualizer seg_viewer;
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
  std::vector < PlaneStruct > *cloudPlanes = new std::vector<PlaneStruct>(); // declare a vector of PlaneStruct-s  ASK??
  PlaneSeg patch_seg;
  //loop thru the patches for to segment each patch (will multitask nxt time)
  for (size_t i= 0 ; i< numberOfPatches ; i++){
    std::cout << "\n From Main:: Cloud Patch size: "<< cloudPatches[i]->points.size () << std::endl;
    patch_seg.patchPlanarSegmentation( cloudPatches[i], cloudPlanes, i);
  }
  // get exception planes
  std::cout << "\n - Running Exception plane segmentation -" << std::endl;
  for (size_t i= 1 ; i< numberOfPatches ; i++){
    if (patch_seg.exceptionPlanarSegmentation( pc2Patches.getExceptionPatch(i), cloudPlanes ) == 1){ 
        std::cout << "exception plane is added with type: " << (*cloudPlanes)[ cloudPlanes->size() - 1 ].type << std::endl;
    };
  }

  // output to visualizer or writer
  seg_viewer.addPlaneStruct( cloudPlanes );
  seg_viewer.runViewer();

  return 0;
}
