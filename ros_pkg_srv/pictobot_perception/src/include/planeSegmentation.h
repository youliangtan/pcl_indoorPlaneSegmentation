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
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <json/json.h>
#include <pcl/filters/radius_outlier_removal.h>


extern Json::Value configParam; // from Json loader


struct PlaneStruct { 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ; 
  std::string type;
  int patchNum;   
  int planeNum;
// collision check : Protrusion
};

// plane Segmentation of Patches
class PlaneSeg {
  private:
    pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals;
    int index;
    int planeNum;
    int patchNum;

  protected:    
    // input pcl normals of a point cloud, compute an average normal 
    int getAverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals);
    // input planar point cloud, obtain string state of a point cloud
    std::string getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);
    // clustering of planes after plane segmentation    
    void planeClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_cloud, std::vector<PlaneStruct> *cloudPlanes);

  public:
    PlaneSeg();
    // input patch of pointcloud
    // pushback plane's pointcloud into a vector of PLaneStruct
    void patchPlanarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<PlaneStruct> *cloudPlanes, int patch_num);
    // Handle to find exception plane (find only one plane for one region)
    // input pointcloud exception patch
    // pushback PlaneStruct into cloudPlanes pointer
    int exceptionPlanarSegmentation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<PlaneStruct> *cloudPlanes);

};

// this is in main cpp
// filtering
void outlinerFiltering (pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);


