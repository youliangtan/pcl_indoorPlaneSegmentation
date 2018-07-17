

#ifndef CLASS_H
#define CLASS_H

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "struct.h"


// PC visualizer
class PCVisualizer{


  public:
  
    pcl::visualization::PCLVisualizer viewer;
    pcl::PCDWriter writer; //init
    int v1, v2;


    PCVisualizer( );

    // view cloud as white for 2 viewers
    void addSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud );

    // visualizer single patch in viewer 2
    void addPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPatch, int index );

    //output vector of PlaneStruct to viewer 2 
    void addPlaneStruct( std::vector < PlaneStruct > *cloudPlanes );

    // way point wall point arrow visualizer
    void addWayPoints( std::vector<pcl::PointXYZ> whole_waypoints, std::vector<pcl::PointXYZ> whole_wallpoints, std::vector<std::vector<PathTrace> > path_traces);

    // run viewer to visualize previous added pointclouds
    void runViewer();

    //data conversion
    pcl::PointXYZ cvtEigen2PclXYZ(Eigen::Vector3f vector);
};


#endif
