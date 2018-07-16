#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <json/json.h>
#include "struct.h"


extern Json::Value configParam; // from Json loader

class PC2Patches{

  private:
    double floorHeight, ceilingHeight, patchHeight;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ;
    

  public:
    // Set Point cloud boundary according to config param
    pcl::PointCloud<pcl::PointXYZ>::Ptr setPCBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);

    //split filtered point cloud to several horizonal patches
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPatch(int patchNum);

    //get exception patches between each patch
    pcl::PointCloud<pcl::PointXYZ>::Ptr getExceptionPatch(int upper_patchNum);

    //get number of patches in filtered point cloud
    int getNumberOfPatches();
};

