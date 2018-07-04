#include "planeSegmentation.h"
#include <pcl/features/normal_3d.h>



// get average normal vector of plane normal
int getAverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int idx){
  int step = 10;
  int count = 0;
  for (size_t i = 0; i < normals->points.size (); i += step){
    if (!std::isnan(normals->points[i].normal_x)) { //normal consisted some with nan
      planes_avgNormals->points[idx].normal_x += normals->points[i].normal_x;
      planes_avgNormals->points[idx].normal_y += normals->points[i].normal_y;
      planes_avgNormals->points[idx].normal_z += normals->points[i].normal_z;
      count++;
    }
  } 
  planes_avgNormals->points[idx].normal_x = planes_avgNormals->points[idx].normal_x/count;
  planes_avgNormals->points[idx].normal_y = planes_avgNormals->points[idx].normal_y/count;
  planes_avgNormals->points[idx].normal_z = planes_avgNormals->points[idx].normal_z/count;

} 


//get state of a plane according to input plane
std::string getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int index){

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (plane_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets with init normals var
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 5cm
  ne.setRadiusSearch (0.6);   // or ne.setKSearch();
  ne.compute (*cloud_normals);   // Compute the features

  //find all plane normal vector and output
  getAverageNormal(cloud_normals, planes_avgNormals, index);
  // std::cout << "avg at func" << planes_avgNormals->points[index] <<std::endl;
  // std::cout << " - [Avg normal] Vector  " << planes_avgNormals->points[index] << std::endl;

  // return appropriate state
  float z_vector = planes_avgNormals->points[index].normal_z;
  if (z_vector > 0.89){ //floor
    return "floor";//3;
  }
  else if (z_vector < -0.89){ //ceiling
    return "ceiling"; //2;
  }
  else if (z_vector < 0.11 && z_vector > -0.11){
    return "wall";//1;
  }
  else{ //others
    return "others";//0;
  }
}




// segment to multiple planes then visualizer
void planarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, int patchNum, std::vector<PlaneStruct> *cloudPlanes){
 
  // var and init
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::cout << " - Running segmentation function for patch " << patchNum << " -" << std::endl;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (configParam["PlanarSegmentation"]["ransacDist"].asDouble());
  seg.setOptimizeCoefficients (true);    // Optional

  // create normal pcl type 
  pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals (new pcl::PointCloud<pcl::Normal>);
  planes_avgNormals->width = 10; // size!!!!! number of planes in model
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;

  double threshpercentage = configParam["PlanarSegmentation"]["cloudFilteredThresh"].asDouble();
  int thresh = cloud_filtered->points.size()*threshpercentage ;
  std::cerr << " - Point Thresh: " << thresh << endl;

  for (size_t idx = 0; cloud_filtered->points.size() > thresh; idx++ ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // segmentation
    std::cerr << "Loop "<< idx  << " | Filtered cloud size: " << cloud_filtered->points.size() << std::endl;
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    // extract from indics
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.filter (*planar_cloud);

    // clusterize each plane (refer to old code)

    // Extract non-plane returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract.setNegative (false);

    // get plane normal then get state
    std::string state = getPlaneState( planar_cloud, planes_avgNormals, idx );  
    
    // ======= OUTPUT TO POINTER =========
    struct PlaneStruct plane_struct;
    plane_struct.cloud = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
    plane_struct.cloud = planar_cloud;
    plane_struct.patchNum = patchNum;
    plane_struct.planeNum = idx;
    plane_struct.type = state;

    cloudPlanes->push_back ( plane_struct );
  }
} 


// Manage exception planes (region between each patches)
int exceptionPlanarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<PlaneStruct> *cloudPlanes){
 
  // var and init
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (configParam["PlanarSegmentation"]["ransacDist"].asDouble());
  seg.setOptimizeCoefficients (true);    // Optional

  // create normal pcl type 
  pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals (new pcl::PointCloud<pcl::Normal>);
  planes_avgNormals->width = 10; // size!!!!! number of planes in model
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // segmentation
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  
  // extract from indics
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.filter (*planar_cloud);

  // clusterize each plane (refer to old code)

  // get plane normal then get state
  std::string state = getPlaneState( planar_cloud, planes_avgNormals );
  if (state  == "wall") { 
    std::cout << "no exception" << std::endl;
    return 0 ; 
  } // return wheres nothing 
  
  // ======= OUTPUT TO POINTER =========
  struct PlaneStruct plane_struct;
  plane_struct.cloud = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
  plane_struct.cloud = planar_cloud;
  plane_struct.patchNum = -1;
  plane_struct.planeNum = -1;
  plane_struct.type = state;

  cloudPlanes->push_back ( plane_struct );
  return 1;
} 