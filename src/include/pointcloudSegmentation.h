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
#include "struct.h"

// filtering of those sparse pointclouds
void outlinerFiltering( pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud );

// main segmentation and visualization of pointcloud
// arg1: input pointcloud, arg2: output planes struct
// return: number of patches
int main_PointCloudSegmentation(  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, std::vector<PlaneStruct> *cloudPlanes );