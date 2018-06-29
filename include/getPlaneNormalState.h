#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

int getAverageNormal(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int idx = 0);

std::string getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int index = 0);