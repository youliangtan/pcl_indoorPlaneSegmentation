#include "planeSegmentation.h"
#include <pcl/features/normal_3d.h>


PlaneSeg::PlaneSeg(){
  std::cout << "INIT FOR PLANESEG" <<  std::endl;

  // create normal pcl type 
  planes_avgNormals = (boost::shared_ptr<pcl::PointCloud<pcl::Normal> >) new pcl::PointCloud<pcl::Normal>() ;
  planes_avgNormals->width = 10; // size!!!!! number of planes in model TODO
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;

  index = 2; //TODO: remove

}

// get average normal vector of plane normal
int PlaneSeg::getAverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals){
  int step = 10;
  int count = 0;

  for (size_t i = 0; i < normals->points.size (); i += step){
    if (!std::isnan(normals->points[i].normal_x)) { //normal consisted some with nan
      planes_avgNormals->points[index].normal_x += normals->points[i].normal_x;
      planes_avgNormals->points[index].normal_y += normals->points[i].normal_y;
      planes_avgNormals->points[index].normal_z += normals->points[i].normal_z;
      count++;
    }
  } 
  planes_avgNormals->points[index].normal_x = planes_avgNormals->points[index].normal_x/count; //TODO
  planes_avgNormals->points[index].normal_y = planes_avgNormals->points[index].normal_y/count;
  planes_avgNormals->points[index].normal_z = planes_avgNormals->points[index].normal_z/count;
} 


//get state of a plane according to input plane
std::string PlaneSeg::getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud){

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
  getAverageNormal(cloud_normals);

  std::cout << "avg at func" << planes_avgNormals->points[index] <<std::endl;
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


void PlaneSeg::planeClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud, std::vector<PlaneStruct> *cloudPlanes ){
  
  // // Create Cluster init
  // Creating the KdTree object for the search method of the extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); // 10cm
  ec.setMinClusterSize ( configParam["PlanarSegmentation"]["planePointsThresh"].asInt() );
  ec.setMaxClusterSize (25000);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // clusterize each plane
  tree->setInputCloud (planar_cloud); //TODO check if theres any use of kd tree func
  ec.setSearchMethod (tree);
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud (planar_cloud);
  ec.extract (cluster_indices);

  std::cout << "Num of Cluster: " << cluster_indices.size () << std::endl;

  // extract and visualize cluster segmentation for each plane
  int clusterNum = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    //create cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (planar_cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //output
    std::cout << "\n- PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    //temp to prevent breakdown
    if (clusterNum== 9) break;

    // get plane normal then get state
    std::string state = getPlaneState( cloud_cluster );  

    // ======= OUTPUT TO POINTER =========
    struct PlaneStruct plane_struct;
    plane_struct.cloud = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
    plane_struct.cloud = cloud_cluster;
    plane_struct.patchNum = patchNum;
    plane_struct.planeNum = planeNum + clusterNum;
    plane_struct.type = state;

    // update patch list datas
    cloudPlanes->push_back ( plane_struct );  
    if (planeMaxIdx_list.size() == patchNum + 1){
      planeMaxIdx_list[patchNum] = planeNum + clusterNum;  
    }
    else{
      planeMaxIdx_list.push_back(planeNum + clusterNum);
    }

    clusterNum ++;
  }
}


// segment to multiple planes then visualizer
void PlaneSeg::patchPlanarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<PlaneStruct> *cloudPlanes, int patch_num){
 
  // var and init
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  patchNum = patch_num;
  std::cout << " - Running segmentation function for patch " << patchNum << " -" << std::endl;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (configParam["PlanarSegmentation"]["ransacDist"].asDouble());
  seg.setOptimizeCoefficients (true);    // Optional

  double threshpercentage = configParam["PlanarSegmentation"]["cloudFilteredThresh"].asDouble();
  int thresh = cloud_filtered->points.size()*threshpercentage ;
  std::cerr << " - Point Thresh: " << thresh << endl;
  int loopThresh = configParam["PlanarSegmentation"]["loopThresh"].asInt();

  for (planeNum = 0; (cloud_filtered->points.size() > thresh) && (planeNum < loopThresh) ; planeNum++ ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // segmentation
    std::cerr << "Loop "<< planeNum  << " | Filtered cloud size: " << cloud_filtered->points.size() << std::endl;
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    // extract from indics
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.filter (*planar_cloud);

    // clusterize each plane (refer to old code)

    // filtering with outlier removal
    outlinerFiltering(planar_cloud);
    std::cout << "after filtered size  "<< planar_cloud->points.size() << std::endl;

    // Extract non-plane returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract.setNegative (false);
    
    // cluster segmentation
    planeClustering(planar_cloud, cloudPlanes);
  }
} 



// Manage exception planes (region between each patches)
int PlaneSeg::exceptionPlanarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<PlaneStruct> *cloudPlanes, int planeIndex){
 
  // var and init
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (configParam["PlanarSegmentation"]["ransacDist"].asDouble());
  seg.setOptimizeCoefficients (true);    // Optional

  // segmentation
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  
  // extract from indics
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.filter (*planar_cloud);

  // get plane normal then get state
  std::string state = getPlaneState( planar_cloud );
  if ((state  == "wall") || (state == "others") ) { 
    std::cout << " - no exception" << std::endl;
    return 0 ; 
  } // return wheres nothing 
  



  
  // ======= OUTPUT TO POINTER =========
  std::cout << " - Exception plane size  "<< planar_cloud->points.size() << std::endl;
  struct PlaneStruct plane_struct;
  plane_struct.cloud = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;
  plane_struct.cloud = planar_cloud;
  plane_struct.type = state;

  // find belonging patchNum and planeNum
  if (state.compare("ceiling") == 0){
    plane_struct.patchNum = planeIndex;
    plane_struct.planeNum = planeMaxIdx_list[planeIndex] + 1;
    std::cout << " - Patch Num > " << plane_struct.patchNum << std::endl;
    std::cout << " - Plane Num > " << plane_struct.planeNum << std::endl;
  }

  else if (state.compare("floor") == 0){
    plane_struct.patchNum = planeIndex -1;
    plane_struct.planeNum = planeMaxIdx_list[planeIndex - 1] + 1;
    std::cout << " - Patch Num > " << plane_struct.patchNum << std::endl;
    std::cout << " - Plane Num > " << plane_struct.planeNum << std::endl;
  }
  else return 0;


  cloudPlanes->push_back ( plane_struct );
  return 1;
} 

