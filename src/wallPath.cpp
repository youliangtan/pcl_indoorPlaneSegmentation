#include "paintPathPlanning.h"

const float pi = 3.14159265;
const float angle_tolerance_h_ = 80.0;//80
const float angle_tolerance_v_ = 15.0;//15
const float painting_length_ = 0.2;
const float min_paint_ratio_ = 0.5;
const float paint_dis_ = 0.3;



WallPathPlan::WallPathPlan() //default
{
    std::cout << "Please input a point cloud." << std::endl;
}

WallPathPlan::WallPathPlan(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    wall_cloud = cloud;
    std::cerr << "Num of points in the cloud: " << wall_cloud.points.size() << std::endl;
}

int WallPathPlan::getWaypoints()
{

    if (wall_cloud.points.size() == 0){
        std::cout << "Empty cloud received!" << std::endl;
        return -1;
    }

    //Compute the plane model using RANSAC
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation object
    seg.setOptimizeCoefficients (true);// Optional
    seg.setModelType (pcl::SACMODEL_PLANE);// Mandatory
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (wall_cloud.makeShared());
    seg.segment (*plane_inliers, *plane_coefficients);
    plane_inlier_indices = *plane_inliers;
    plane_coeff = *plane_coefficients;
    plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];

    if (plane_inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    if(plane_normal[0] < 0){ //to adjust and control the wall normal
        plane_normal = -plane_normal;
    }

    //Points projection onto the plane model
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (wall_cloud.makeShared());
    proj.setModelCoefficients (plane_coefficients);
    proj.filter (wall_projected);

    //Corner Detection Method based on Harris
    keypoints = planeHarrisCorenerDetection(wall_projected);
    std::cerr << "Keypoints num: " << keypoints.points.size() << std::endl;

    //Refine and sort the plane corners
    int root_corner_index = topLeftCorner(keypoints);
    cornerSort(keypoints, root_corner_index, -1, angle_tolerance_h_, angle_tolerance_v_, corners);
    std::cerr << "Sorted corners size: " << corners.size() << std::endl;
    for (size_t i = 0; i < corners.size(); i++){
        std:: cerr << "Sorted corner " << i << " : " << corners[i] << std::endl;
    }

    //Define the horizontal and vertical vectors within the wall plane
    planeVectorCal(corners, *plane_coefficients, angle_tolerance_h_, angle_tolerance_v_, plane_vector_h, plane_vector_v);
    std::cerr << "Plane vector horizontal: " << plane_vector_h[0] << " " << plane_vector_h[1] << " " << plane_vector_h[2] << std::endl;
    std::cerr << "Plane vector vertical: " << plane_vector_v[0] << " " << plane_vector_v[1] << " " << plane_vector_v[2] << std::endl;

    //Construct horizontal and parallelly near-vertical edges between the corners
    std::vector<pcl::ModelCoefficients> edges_coeff; //edges_coeff
    edgeCoeffCal(corners, plane_vector_h, plane_vector_v, angle_tolerance_h_, angle_tolerance_v_, edges_coeff);
    std::cerr << "Edge coeff size: " << edges_coeff.size() << std::endl;


    //Extract the edge intersection points as the refined corners
    cornerAdjust(edges_coeff, corners_refined);
    std::cerr << "Corner refined size: " << corners_refined.size() << std::endl;

    //Generate the waypoints
    std::vector<pcl::PointXYZ> way_points;
    if(pathPlanParall(corners_refined, way_points, true) == -1){
        std::cerr << "Fail to generate waypoints!" << std::endl
                  << "Using the back up methods" << std::endl;

        if(backUpMethod() == -1){
            return -1;
        }
    }
    else{
        std::cerr << "Num of waypoints: " << way_points.size() << std::endl;
        wallpoints = way_points;

        //Generate the painting points
        for(size_t i = 0; i < way_points.size(); i++){
            way_points[i].x = way_points[i].x - paint_dis_ * plane_normal[0];
            way_points[i].y = way_points[i].y - paint_dis_ * plane_normal[1];
            way_points[i].z = way_points[i].z - paint_dis_ * plane_normal[2];
        }
        waypoints = way_points;

        return 0;
    }
}

int WallPathPlan::backUpMethod(){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corners (new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<int> cloud_corners_indices;
    cloud_corners_indices.push_back(topLeftCorner(keypoints));
    cloud_corners_indices.push_back(topRightCorner(keypoints));
    cloud_corners_indices.push_back(bottomLeftCorner(keypoints));
    cloud_corners_indices.push_back(bottomRightCorner(keypoints));

    for(size_t i = 0; i < 4; i++){
        cloud_corners->points.push_back(keypoints.points[cloud_corners_indices[i]]);
    }

    std::cerr<< "Keypoints size: " << keypoints.points.size() << std::endl;
    std::cerr<< "cloud corner size: " << cloud_corners->points.size() << std::endl;
    std::cerr << "Value" << cloud_corners->points[0] << std::endl
                 << cloud_corners->points[1] << std::endl
                    << cloud_corners->points[2] << std::endl
                       << cloud_corners->points[3] << std::endl;

    corners.clear();
    corners_refined.clear();

    cornerSort(*cloud_corners, 0, -1, angle_tolerance_h_, angle_tolerance_v_, corners);
    planeVectorCal(corners, plane_coeff, angle_tolerance_h_, angle_tolerance_v_, plane_vector_h, plane_vector_v);
    std::vector<pcl::ModelCoefficients> edges_coeff;
    edgeCoeffCal(corners, plane_vector_h, plane_vector_v, angle_tolerance_h_, angle_tolerance_v_, edges_coeff);
    cornerAdjust(edges_coeff, corners_refined);

    std::vector<pcl::PointXYZ> way_points;
    if(pathPlanParall(corners_refined, way_points, true) == -1){
        std::cerr << "Fail to generate waypoints!" << std::endl
                  << "Back up method also fail!" << std::endl;
        return -1;

    }
    std::cerr << "Num of waypoints: " << way_points.size() << std::endl;
    wallpoints = way_points;

    //Generate the painting points
    for(size_t i = 0; i < way_points.size(); i++){
        way_points[i].x = way_points[i].x - paint_dis_ * plane_normal[0];
        way_points[i].y = way_points[i].y - paint_dis_ * plane_normal[1];
        way_points[i].z = way_points[i].z - paint_dis_ * plane_normal[2];
    }
    waypoints = way_points;

    return 0;
}

pcl::PointCloud<pcl::PointXYZI> WallPathPlan::planeHarrisCorenerDetection(pcl::PointCloud<pcl::PointXYZ> plane){

    //Generate the convex hull
    pcl::PointCloud<pcl::PointXYZ> hull;
    std::vector<pcl::Vertices> hull_vertices;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (plane.makeShared());
    chull.setAlpha (0.05);//0.1 //0.05 //0.03
    chull.reconstruct (hull, hull_vertices);
    std::cerr << "Concave hull: " << hull.points.size () << std::endl;

    int cube_height = 40;
    long plane_cube_size = 2 * plane.points.size() + (cube_height-1) * hull.points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cube_cloud -> width = plane_cube_size;
    cube_cloud -> height = 1;
    cube_cloud -> points.resize(cube_cloud -> width * cube_cloud -> height);

    long count = 0;
    float unit_height = 0.01;
    for (size_t i = 0; i < (cube_height + 1); i++){
        if (i == 0 || i == cube_height){
            for (size_t j = 0; j < plane.points.size(); j++){
                cube_cloud->points[count].x = plane.points[j].x + i * plane_normal[0] * unit_height;
                cube_cloud->points[count].y = plane.points[j].y + i * plane_normal[1] * unit_height;
                cube_cloud->points[count].z = plane.points[j].z + i * plane_normal[2] * unit_height;
                count++;
            }
        } else{
            for (size_t j = 0; j < hull.points.size(); j++){
                cube_cloud->points[count].x = hull.points[j].x + i * plane_normal[0] * unit_height;
                cube_cloud->points[count].y = hull.points[j].y + i * plane_normal[1] * unit_height;
                cube_cloud->points[count].z = hull.points[j].z + i * plane_normal[2] * unit_height;
                count++;
            }
        }
    }
    std::cout << "count: " << count <<std::endl;

    //Apply Harris Corner Detection Algorithem
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius (0.08); //0.01
    detector.setInputCloud(cube_cloud);
    detector.setThreshold(0.01); //0.01
    //detector.setRefine(true);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    detector.compute (*keypoints);
    std::cerr << "Keypoints size: " << keypoints -> points.size() << std::endl;

    //filtering the duplicate points
    pcl::UniformSampling<pcl::PointXYZI> uniform_sampling;
    uniform_sampling.setInputCloud (keypoints);
    uniform_sampling.setRadiusSearch (0.01);//model_ss_
    uniform_sampling.filter (*keypoints);
    std::cout << "Model total points: " << keypoints->size () << std::endl;

    //Identify the corners lying on the wall plane
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    *plane_coefficients = plane_coeff;

    pcl::ModelOutlierRemoval<pcl::PointXYZI> plane_filter;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    plane_filter.setModelCoefficients (*plane_coefficients);
    plane_filter.setThreshold(0.03);
    plane_filter.setModelType(pcl::SACMODEL_PLANE);
    plane_filter.setInputCloud(keypoints);
    plane_filter.filter(*keypoints_filtered);
    std::cerr << "Keypoints size after filtering: " << keypoints_filtered -> points.size() << std::endl;

    pcl::ProjectInliers<pcl::PointXYZI> proj_corner;
    proj_corner.setModelType (pcl::SACMODEL_PLANE);
    //proj.setIndices (inliers);
    proj_corner.setInputCloud (keypoints_filtered);
    proj_corner.setModelCoefficients (plane_coefficients);
    proj_corner.filter (*keypoints_filtered);

    return *keypoints_filtered;
}

int WallPathPlan::topLeftCorner(pcl::PointCloud<pcl::PointXYZI> corners){ //depends on the scaner coordnate frame
    if (corners.points.size() != 0){
    } else {
        std::cerr << "Errors! Empty clouds!" << std::endl;
        return -1;
    }
    float max = corners.points[0].y + corners.points[0].z;
    int max_index = 0;
    float check = 0;
    for (size_t i = 1; i < corners.points.size(); i++){
        check = corners.points[i].y + corners.points[i].z;
        if (check > max){
            max = check;
            max_index = i;
        }
    }
    return max_index;
}

int WallPathPlan::topRightCorner(pcl::PointCloud<pcl::PointXYZI> corners){ //depends on the scaner coordnate frame
    if (corners.points.size() != 0){
    } else {
        std::cerr << "Errors! Empty clouds!" << std::endl;
        return -1;
    }
    float max = -corners.points[0].y + corners.points[0].z;
    int max_index = 0;
    float check = 0;
    for (size_t i = 1; i < corners.points.size(); i++){
        check = -corners.points[i].y + corners.points[i].z;
        if (check > max){
            max = check;
            max_index = i;
        }
    }
    return max_index;
}

int WallPathPlan::bottomLeftCorner(pcl::PointCloud<pcl::PointXYZI> corners){ //depends on the scaner coordnate frame
    if (corners.points.size() != 0){
    } else {
        std::cerr << "Errors! Empty clouds!" << std::endl;
        return -1;
    }
    float max = corners.points[0].y - corners.points[0].z;
    int max_index = 0;
    float check = 0;
    for (size_t i = 1; i < corners.points.size(); i++){
        check = corners.points[i].y - corners.points[i].z;
        if (check > max){
            max = check;
            max_index = i;
        }
    }
    return max_index;
}

int WallPathPlan::bottomRightCorner(pcl::PointCloud<pcl::PointXYZI> corners){ //depends on the scaner coordnate frame
    if (corners.points.size() != 0){
    } else {
        std::cerr << "Errors! Empty clouds!" << std::endl;
        return -1;
    }
    float max = -corners.points[0].y - corners.points[0].z;
    int max_index = 0;
    float check = 0;
    for (size_t i = 1; i < corners.points.size(); i++){
        check = -corners.points[i].y - corners.points[i].z;
        if (check > max){
            max = check;
            max_index = i;
        }
    }
    return max_index;
}

int WallPathPlan::vectDirect(Eigen::Vector3f edge_vector, float angle_tolerance_h, float angle_tolerance_v){
    if (std::abs(edge_vector[2]) < std::cos(angle_tolerance_h * pi /180)){ //check for the horizontal direction
        return -1;
    } else if(std::abs(edge_vector[2]) > std::cos(angle_tolerance_v * pi /180)){ //check for the vertical direction
        return 1;
    } else{
        return 0;
    }
}

bool WallPathPlan::sortCornerDistance(const plane_edge edge_1, const plane_edge edge_2){
    return edge_1.length < edge_2.length;
}

void WallPathPlan::cornerSort(pcl::PointCloud<pcl::PointXYZI> corners, int root_index, int direction, float angle_tolerance_h, float angle_tolerance_v, std::vector<pcl::PointXYZ> &corner_sorted){

    pcl::PointXYZ corner;
    corner.x = corners.points[root_index].x;
    corner.y = corners.points[root_index].y;
    corner.z = corners.points[root_index].z;
    corner_sorted.push_back(corner);// save the corner in the new sorting point cloud

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    inliers->indices.push_back(root_index);
    extract.setInputCloud(corners.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(corners); // remove the root corner from the old one

    if (corners.points.size() == 0){
        return;
    }

    //int direction = -1; // 0: tilted; 1:vertical; -1:horizontal; starting from horizontal direction
    std::vector<plane_edge> edges_dir1;
    std::vector<plane_edge> edges_dir2;
    plane_edge edge;
    for (size_t i = 0; i < corners.points.size(); i++){ //calculate the edge vector value, length and define their directions
        edge.length = std::sqrt(std::pow(corner.x - corners.points[i].x,2) +
                                std::pow(corner.y - corners.points[i].y,2) +
                                std::pow(corner.z - corners.points[i].z,2));
        edge.line_vector[0] = (corners.points[i].x - corner.x) / edge.length;
        edge.line_vector[1] = (corners.points[i].y - corner.y) / edge.length;
        edge.line_vector[2] = (corners.points[i].z - corner.z) / edge.length;
        edge.direction = vectDirect(edge.line_vector, angle_tolerance_h, angle_tolerance_v);

        if(edge.direction == direction){
            edge.index = i;
            edge.start_point[0] = corner.x;
            edge.start_point[1] = corner.y;
            edge.start_point[2] = corner.z;
            edge.end_point[0] = corners.points[i].x;
            edge.end_point[1] = corners.points[i].y;
            edge.end_point[2] = corners.points[i].z;
            edges_dir1.push_back(edge);
        } else if (edge.direction == - direction){
            edge.index = i;
            edge.start_point[0] = corner.x;
            edge.start_point[1] = corner.y;
            edge.start_point[2] = corner.z;
            edge.end_point[0] = corners.points[i].x;
            edge.end_point[1] = corners.points[i].y;
            edge.end_point[2] = corners.points[i].z;
        } else{
            continue;
        }
        edge = {};
    }

    int root_index_new;
    if (edges_dir1.size() != 0){
        std::sort(edges_dir1.begin(), edges_dir1.end(), WallPathPlan::sortCornerDistance);
        root_index_new = edges_dir1[0].index;
        edges_dir1.clear();

        cornerSort(corners, root_index_new, - direction, angle_tolerance_h, angle_tolerance_v, corner_sorted);
    } else if(edges_dir2.size() != 0){
        direction = - direction;
        std::sort(edges_dir2.begin(), edges_dir2.end(), WallPathPlan::sortCornerDistance);
        root_index_new = edges_dir2[0].index;
        edges_dir2.clear();

        cornerSort(corners, root_index_new, - direction, angle_tolerance_h, angle_tolerance_v, corner_sorted);
    } else {//for future debug
        std::cout << "Fails to find the horizontal or vertical lines!" << std::endl;
    }
}

void WallPathPlan::planeVectorCal(std::vector<pcl::PointXYZ> corners_in, pcl::ModelCoefficients plane_coef, float angle_tolerance_h,
                    float angle_tolerance_v, Eigen::Vector3f &plane_vector_h, Eigen::Vector3f &plane_vector_v){
    /*To calculate the horizontal and th near-vertical vectors in the plane.
    Take note that all the near-vertical vectors are parallel, resulting in
    only two vectors as the function output*/

    //horizontal line vector calculation
    float a = plane_coef.values[0];
    float b = plane_coef.values[1];
    float c = plane_coef.values[2];
    float r = 0;
    plane_vector_h[1] = (c*r*b + a*std::sqrt(std::pow(b,2)+std::pow(a,2)+std::pow(a,2)*std::pow(r,2)-std::pow(b,2)*std::pow(r,2)-std::pow(c,2)*std::pow(r,2)))/(std::pow(a,2)+std::pow(b,2));
    plane_vector_h[0] = (c*r-b*plane_vector_h[1])/a;
    plane_vector_h[2] = r;
    plane_vector_h.normalize();
    if (plane_vector_h[1] > 0){
        plane_vector_h = - plane_vector_h;
    }

    //near vertical line vector calculation
    std::vector<Eigen::Vector3f> near_vert_vector;
    Eigen::Vector3f edge_vector;
    float edge_length;
    int corner_index_1 = 0;
    int corner_index_2 = 0;
    for (size_t i = 0; i < corners_in.size(); i++){
        corner_index_1 = i;
        corner_index_2 = i + 1;
        if (i == corners_in.size() - 1){
            corner_index_2 = 0;
        }

        edge_length = std::sqrt(std::pow(corners_in[i+1].x - corners_in[i].x,2) +
                                std::pow(corners_in[i+1].y - corners_in[i].y,2) +
                                std::pow(corners_in[i+1].z - corners_in[i].z,2));
        edge_vector[0] = (corners_in[i+1].x - corners_in[i].x) / edge_length;
        edge_vector[1] = (corners_in[i+1].y - corners_in[i].y) / edge_length;
        edge_vector[2] = (corners_in[i+1].z - corners_in[i].z) / edge_length;

        if(std::abs(edge_vector[2]) > std::cos(angle_tolerance_v * pi /180)){
            near_vert_vector.push_back(edge_vector);
        }
        //std::cerr << "Edge_vector: " << edge_vector << std::endl;
    }

    plane_vector_v = Eigen::Vector3f::Zero();
    //std::cout <<std::endl << "Plane vector test: " << std::endl << plane_vector_v << std::endl;

    for (size_t i = 0; i < near_vert_vector.size(); i++){
        if (near_vert_vector[i][2] < 0){
            near_vert_vector[i][0] = -near_vert_vector[i][0];
            near_vert_vector[i][1] = -near_vert_vector[i][1];
            near_vert_vector[i][2] = -near_vert_vector[i][2];
        }
        plane_vector_v[0] += near_vert_vector[i][0];
        plane_vector_v[1] += near_vert_vector[i][1];
        plane_vector_v[2] += near_vert_vector[i][2];
    }
    plane_vector_v.normalize();
}

void WallPathPlan::edgeCoeffCal(std::vector<pcl::PointXYZ> corners_in, Eigen::Vector3f plane_vector_h, Eigen::Vector3f plane_vector_v, float angle_tolerance_h,
             float angle_tolerance_v, std::vector<pcl::ModelCoefficients> &edges_coeff){
    pcl::ModelCoefficients edge_coeff;
    edge_coeff.values.clear();

    std::vector<Eigen::Vector3f> edge_vectors;
    edgeVectCal(corners_in, edge_vectors);

    int corner_index_1 = 0;
    int corner_index_2 = 0;
    for (size_t i = 0; i < corners_in.size(); i++){
        corner_index_1 = i;
        corner_index_2 = i+1;
        if (i == corners_in.size() - 1){
            corner_index_2 = 0;
        }
            if (std::abs(edge_vectors[i][2]) < std::cos(angle_tolerance_h * pi /180)) { //horizontal
                edge_coeff.values.push_back((corners_in[corner_index_1].x + corners_in[corner_index_2].x) / 2);
                edge_coeff.values.push_back((corners_in[corner_index_1].y + corners_in[corner_index_2].y) / 2);
                edge_coeff.values.push_back((corners_in[corner_index_1].z + corners_in[corner_index_2].z) / 2);
                edge_coeff.values.push_back(plane_vector_h[0]);
                edge_coeff.values.push_back(plane_vector_h[1]);
                edge_coeff.values.push_back(plane_vector_h[2]);

                edges_coeff.push_back(edge_coeff);
                edge_coeff.values.clear();
            } else if (std::abs(edge_vectors[i][2]) > std::cos(angle_tolerance_v * pi /180)) { //vertical
                edge_coeff.values.push_back((corners_in[corner_index_1].x + corners_in[corner_index_2].x) / 2);
                edge_coeff.values.push_back((corners_in[corner_index_1].y + corners_in[corner_index_2].y) / 2);
                edge_coeff.values.push_back((corners_in[corner_index_1].z + corners_in[corner_index_2].z) / 2);
                edge_coeff.values.push_back(plane_vector_v[0]);
                edge_coeff.values.push_back(plane_vector_v[1]);
                edge_coeff.values.push_back(plane_vector_v[2]);

                edges_coeff.push_back(edge_coeff);
                edge_coeff.values.clear();
            } else {
                std::cerr << "Fails to construct a horizontal or vertical edge!" <<std::endl;
            }
    }
}

void WallPathPlan::edgeVectCal(std::vector<pcl::PointXYZ> corner_sorted, std::vector<Eigen::Vector3f> &edge_vectors){
    int corner_index1 = 0;
    int corner_index2 = 0;
    Eigen::Vector3f edge_vector = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < corner_sorted.size(); i++){
        corner_index1 = i;
        corner_index2 = i + 1;
        if (i == corner_sorted.size() - 1){
            corner_index2 = 0;
        }
        edge_vector[0] = corner_sorted[corner_index2].x - corner_sorted[corner_index1].x;
        edge_vector[1] = corner_sorted[corner_index2].y - corner_sorted[corner_index1].y;
        edge_vector[2] = corner_sorted[corner_index2].z - corner_sorted[corner_index1].z;
        edge_vector.normalize();
        edge_vectors.push_back(edge_vector);
    }
}

void WallPathPlan::cornerAdjust(std::vector<pcl::ModelCoefficients> edges_coeff, std::vector<pcl::PointXYZ> &corners_adjusted){

    Eigen::Vector4f intersect;
    pcl::PointXYZ corner;

    int edge_index1 = 0;
    int edge_index2 = 0;
    for (size_t i = 0; i < edges_coeff.size(); i++){
        edge_index1 = i-1;
        edge_index2 = i;
        if (i == 0){
            edge_index1 = edges_coeff.size() - 1;
        }

        if (!pcl::lineWithLineIntersection (edges_coeff[edge_index1], edges_coeff[edge_index2], intersect))
         {
           PCL_ERROR("\nCould not estimate a line with line intersection\n");
           //return (-1);
         } else {
            corner.x = intersect[0];
            corner.y = intersect[1];
            corner.z = intersect[2];
            corners_adjusted.push_back(corner);
        }
    }
}

int WallPathPlan::pathPlanParall(std::vector<pcl::PointXYZ> corners_adjusted, std::vector<pcl::PointXYZ> &waypoints, bool paint_direction ){
    //corners_adjsted containing points which form a parallelogram within a plane

    // edge_1 & edge_2 are two vectors in the plane; used to calculate the plane normal vectors;
    //Take note that edge_1 must be horizontal;
    //direction true means left to right
    if (corners_adjusted.size() != 4){
        return -1;
    }

    std::vector<pcl::ModelCoefficients> edges_coeff;
    edgeCoeffCal(corners_adjusted, plane_vector_h, plane_vector_v, angle_tolerance_h_, angle_tolerance_v_, edges_coeff);
    if (lineWithLineDisCompute(edges_coeff[0], edges_coeff[2]) < painting_length_ * min_paint_ratio_){
        return 1;
    } else {
        if (paint_direction) {
            waypoints.push_back(corners_adjusted[0]);
            waypoints.push_back(corners_adjusted[1]);
            trace_vector.push_back(plane_vector_h);
        } else{
            waypoints.push_back(corners_adjusted[1]);
            waypoints.push_back(corners_adjusted[0]);
            trace_vector.push_back(-plane_vector_h);
        }
    }

    std::vector<Eigen::Vector3f> edge_vectors;
    edgeVectCal(corners_adjusted, edge_vectors);

    int edge_index1;
    int edge_index2;
    Eigen::Vector3f edge_1;
    Eigen::Vector3f edge_2;
    for (size_t i = 0; i < edge_vectors.size(); i++){
        if (i == edge_vectors.size() - 1){
            edge_index1 = i;
            edge_index2 = 0;
        } else {
            edge_index1 = i;
            edge_index2 = i + 1;
        }

        if (vectDirect(edge_vectors[edge_index1], angle_tolerance_h_, angle_tolerance_v_) == -1 &&
                vectDirect(edge_vectors[edge_index2], angle_tolerance_h_, angle_tolerance_v_) == 1){//check for edge slope
            edge_1 = edge_vectors[edge_index1];
            edge_2 = edge_vectors[edge_index2];
            break;
        }
    }

    Eigen::Vector3f plane_normal_vector = edge_1.cross(edge_2);
    Eigen::Vector3f plane_downward_vector = plane_normal_vector.cross(edge_1);
    plane_downward_vector.normalize();
    if (plane_downward_vector[2] > 0){
        plane_downward_vector = - plane_downward_vector;
    }
    //std::cerr << "plane_downward_vector: " << std::endl << plane_downward_vector <<std::endl;

    pcl::PointXYZ check_point;
    check_point.x = corners_adjusted[0].x + plane_downward_vector[0] * painting_length_;
    check_point.y = corners_adjusted[0].y + plane_downward_vector[1] * painting_length_;
    check_point.z = corners_adjusted[0].z + plane_downward_vector[2] * painting_length_;

    pcl::ModelCoefficients imaginary_edge_coeff;
    imaginary_edge_coeff.values.push_back(check_point.x);
    imaginary_edge_coeff.values.push_back(check_point.y);
    imaginary_edge_coeff.values.push_back(check_point.z);
    imaginary_edge_coeff.values.push_back(plane_vector_h[0]);
    imaginary_edge_coeff.values.push_back(plane_vector_h[1]);
    imaginary_edge_coeff.values.push_back(plane_vector_h[2]);
    edges_coeff[0] = imaginary_edge_coeff;

    std::vector<pcl::PointXYZ> imaginary_corners;
    cornerAdjust(edges_coeff,imaginary_corners);

    pathPlanParall(imaginary_corners, waypoints, !paint_direction);
}

float WallPathPlan::lineWithLineDisCompute(pcl::ModelCoefficients line_1, pcl::ModelCoefficients line_2){
    Eigen::Vector3f parallel_vector (line_1.values[3], line_1.values[4], line_1.values[5]);
    //parallel_vector.normalize();

    Eigen::Vector3f intersect_vector (line_1.values[0] - line_2.values[0],
                                      line_1.values[1] - line_2.values[1],
                                      line_1.values[2] - line_2.values[2]);
    Eigen::Vector3f plane_normal = parallel_vector.cross(intersect_vector);
    Eigen::Vector3f line_normal_in_plane = plane_normal.cross(parallel_vector);
    line_normal_in_plane.normalize();

    pcl::ModelCoefficients normal_line_coeff;
    normal_line_coeff.values.push_back(line_1.values[0]);
    normal_line_coeff.values.push_back(line_1.values[1]);
    normal_line_coeff.values.push_back(line_1.values[2]);
    normal_line_coeff.values.push_back(line_normal_in_plane[0]);
    normal_line_coeff.values.push_back(line_normal_in_plane[1]);
    normal_line_coeff.values.push_back(line_normal_in_plane[2]);

    Eigen::Vector4f intersect1;
    Eigen::Vector4f intersect2;
    pcl::lineWithLineIntersection (line_1, normal_line_coeff, intersect1);
    pcl::lineWithLineIntersection (line_2, normal_line_coeff, intersect2);

    float distance = std::sqrt(std::pow(intersect1[0] - intersect2[0], 2) +
                                std::pow(intersect1[1] - intersect2[1], 2) +
                                std::pow(intersect1[2] - intersect2[2], 2));

    return distance;
}
