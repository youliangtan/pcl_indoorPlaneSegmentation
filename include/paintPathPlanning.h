#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/intersections.h>
#include <pcl/common/distances.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_2d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>

#include "struct.h"
#include "visualizer.h"


void pathIntegrate(std::vector<PlaneStruct> input_patches, int patch_size, std::vector<std::vector<PathTrace> > &pathTraces, PCVisualizer &path_viewer);

class WallPathPlan{
    public:
	pcl::PointCloud<pcl::PointXYZ> wall_cloud;
        pcl::PointCloud<pcl::PointXYZ> wall_projected;
        pcl::PointCloud<pcl::PointXYZ> wall_hull;
        pcl::PointCloud<pcl::PointXYZI> keypoints;
        pcl::PointIndices plane_inlier_indices;
        pcl::ModelCoefficients plane_coeff;
        Eigen::Vector3f plane_normal;
        Eigen::Vector3f plane_vector_h;
        Eigen::Vector3f plane_vector_v;
        std::vector<pcl::PointXYZ> corners;
        std::vector<pcl::PointXYZ> corners_refined;
	std::vector<pcl::PointXYZ> wallpoints;
        std::vector<pcl::PointXYZ> waypoints;
	std::vector<Eigen::Vector3f> trace_vector;

	WallPathPlan();

	WallPathPlan(pcl::PointCloud<pcl::PointXYZ> cloud);

	int getWaypoints();

	int backUpMethod();

	pcl::PointCloud<pcl::PointXYZI> planeHarrisCorenerDetection(pcl::PointCloud<pcl::PointXYZ> plane);

	int topLeftCorner(pcl::PointCloud<pcl::PointXYZI> corners);

	int topRightCorner(pcl::PointCloud<pcl::PointXYZI> corners);

	int bottomLeftCorner(pcl::PointCloud<pcl::PointXYZI> corners);

	int bottomRightCorner(pcl::PointCloud<pcl::PointXYZI> corners);

	int vectDirect(Eigen::Vector3f edge_vector, float angle_tolerance_h, float angle_tolerance_v);

	static bool sortCornerDistance(const plane_edge edge_1, const plane_edge edge_2);

	void cornerSort(pcl::PointCloud<pcl::PointXYZI> corners, int root_index, int direction, float angle_tolerance_h, float angle_tolerance_v, std::vector<pcl::PointXYZ> &corner_sorted);

	void planeVectorCal(std::vector<pcl::PointXYZ> corners_in, pcl::ModelCoefficients plane_coef, float angle_tolerance_h, float angle_tolerance_v, Eigen::Vector3f &plane_vector_h, Eigen::Vector3f &plane_vector_v);

	void edgeCoeffCal(std::vector<pcl::PointXYZ> corners_in, Eigen::Vector3f plane_vector_h, Eigen::Vector3f plane_vector_v, float angle_tolerance_h, float angle_tolerance_v, std::vector<pcl::ModelCoefficients> &edges_coeff);

	void edgeVectCal(std::vector<pcl::PointXYZ> corner_sorted, std::vector<Eigen::Vector3f> &edge_vectors);

	void cornerAdjust(std::vector<pcl::ModelCoefficients> edges_coeff, std::vector<pcl::PointXYZ> &corners_adjusted);

	int pathPlanParall(std::vector<pcl::PointXYZ> corners_adjusted, std::vector<pcl::PointXYZ> &waypoints, bool paint_direction = true);

	float lineWithLineDisCompute(pcl::ModelCoefficients line_1, pcl::ModelCoefficients line_2);

};

class CeilPath{
    public:
	std::string type;
	pcl::PointCloud<pcl::PointXYZ> wall_cloud;
	pcl::PointCloud<pcl::PointXYZ> ceil_cloud;
	pcl::PointCloud<pcl::PointXYZ> wall_cloud_updated;
	pcl::PointIndices ceil_inliers_indices;
	pcl::PointIndices wall_inliers_indices;
	pcl::ModelCoefficients ceil_coeff;
	pcl::ModelCoefficients wall_coeff;
	pcl::ModelCoefficients edge_coeff;
	Eigen::Vector3f ceil_normal;
	Eigen::Vector3f wall_normal;
	Eigen::Vector3f paint_dir;
	pcl::PointXYZ edge_end1, edge_end2;
	pcl::PointXYZ start_point, end_point;
	std::vector<pcl::PointXYZ> waypoints;
	Eigen::Vector3f trace_vector;

	CeilPath();

	CeilPath(pcl::PointCloud<pcl::PointXYZ> wall, pcl::PointCloud<pcl::PointXYZ> ceil, std::string type_name);

	void getWaypoints();

	void planeSeg(const pcl::PointCloud<pcl::PointXYZ> cloud, const float thresh, pcl::ModelCoefficients &coeffs, pcl::PointIndices &inliers );

	void cvtPlaneCoeff2Vect(pcl::ModelCoefficients coeffs, Eigen::Vector4f &vector);

	void cvtLineVect2Coeff(Eigen::VectorXf vector, pcl::ModelCoefficients &coeffs);

	void lineEndPointCompute(pcl::PointCloud<pcl::PointXYZ> cloud, char field_name, pcl::PointXYZ &minp, pcl::PointXYZ &maxp);

};
