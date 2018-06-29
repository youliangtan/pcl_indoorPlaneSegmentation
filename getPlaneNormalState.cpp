#include "getPlaneNormalState.h"
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
  ne.setRadiusSearch (0.5);   // or ne.setKSearch();
  // Compute the features
  ne.compute (*cloud_normals);

  //find all plane normal vector and output
  getAverageNormal(cloud_normals, planes_avgNormals, index);
  // writer.write<pcl::Normal> ("normal_vector.pcd", *cloud_normals, false);
  std::cout << "avg at func" << planes_avgNormals->points[index] <<std::endl;
  std::cout << " - [Avg normal] Vector  " << planes_avgNormals->points[index] << std::endl;

  //check cluster index via output
  // if (index == 6){
  //   writer.write<pcl::PointXYZ> ("./pcd/output_cluster.pcd", *plane_cloud, false);
  //   viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (plane_cloud, cloud_normals, 15, 0.28, "normals");
  // }      


  // return appropriate state
  float z_vector = planes_avgNormals->points[index].normal_z;
  if (z_vector > 0.92){ //floor
    return "floor";//3;
  }
  else if (z_vector < -0.92){ //ceiling
    return "ceiling"; //2;
  }
  else if (z_vector < 0.08 && z_vector > -0.08){
    return "wall";//1;
  }
  else{ //others
    return "others";//0;
  }
}