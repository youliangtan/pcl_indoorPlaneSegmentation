#ifndef STRUCT_H
#define STRUCT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


struct plane_edge {
    int index;
    Eigen::Vector3f line_vector;
    float length;
    int direction; //0: tilted; 1: vertical; -1: horizontal;
    Eigen::Vector3f start_point;
    Eigen::Vector3f end_point;

};

struct PlaneStruct {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::string type;
  int patchNum;
  int planeNum;
};

struct PathTrace{
    Eigen::Vector3f start_point;
    Eigen::Vector3f end_point;
    Eigen::Vector3f normal_vector;
    Eigen::Vector3f trace_vector;
};

#endif