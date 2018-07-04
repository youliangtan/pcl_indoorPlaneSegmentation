#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

// input pcl normals of a point cloud, compute an average normal 
int getAverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int idx = 0);

// input planar point cloud, obtain string state of a point cloud
// state: ceiling, floor, wall, others
std::string getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int index = 0);

