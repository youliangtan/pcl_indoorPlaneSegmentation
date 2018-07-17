#include "paintPathPlanning.h"

const float plane_thresh_ransac_ = 0.02;
const float edge_paint_dis_ = 0.3;
const float edge_wall_cut_dis_ = 0.2;

CeilPath::CeilPath()
{
    std::cout << "Please input the ceilling and wall coud. " <<std::endl;
}

CeilPath::CeilPath(pcl::PointCloud<pcl::PointXYZ> wall, pcl::PointCloud<pcl::PointXYZ> ceil, std::string type_name)
{
    wall_cloud = wall;
    ceil_cloud = ceil;
    type = type_name;
    std::cout << "Wall cloud size: " << wall.points.size() << std::endl
                 << "Ceilling cloud size: " << ceil.points.size() << std::endl;
    std::cout << "Type: " << type_name << std::endl;
}

void CeilPath::getWaypoints()
{
    /*Determine the plane models for the celing and the wall*/
    pcl::ModelCoefficients::Ptr ceil_coefficients (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr wall_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr ceil_inliers (new pcl::PointIndices);
    pcl::PointIndices::Ptr wall_inliers (new pcl::PointIndices);
    planeSeg(ceil_cloud, plane_thresh_ransac_, *ceil_coefficients, *ceil_inliers);
    planeSeg(wall_cloud, plane_thresh_ransac_, *wall_coefficients, *wall_inliers); //wall
    ceil_coeff = *ceil_coefficients;
    wall_coeff = *wall_coefficients;

    /*Determin the planes normals of the wall and the ceiling*/
    for (size_t i = 0; i < 3; i++){
        ceil_normal[i] = ceil_coefficients -> values[i];
        wall_normal[i] = wall_coefficients -> values[i];
    }

    if (type.compare("ceiling") == 0){
        if (ceil_normal[2] < 0){
            for (size_t  i = 0; i < 3; i++){
                ceil_normal[i] = - ceil_normal[i];
            }
        }
    } else if (type.compare("floor") == 0){
        if (ceil_normal[2] > 0){
            for (size_t  i = 0; i < 3; i++){
                ceil_normal[i] = - ceil_normal[i];
            }
        }
    }

    if (wall_normal[0] < 0){
        for (size_t  i = 0; i < 3; i++){
            wall_normal[i] = - wall_normal[i];
        }
    }
    paint_dir = (ceil_normal + wall_normal);
    paint_dir.normalize();
    std::cerr << "Painting direction: " << paint_dir[0] << " "
                                        << paint_dir[1] << " "
                                        << paint_dir[2] << std::endl;

    /*Plane intersection calculation*/
    Eigen::Vector4f ceil_vect;
    Eigen::Vector4f wall_vect;
    Eigen::VectorXf intersection_line;
    pcl::ModelCoefficients::Ptr line_coeff (new pcl::ModelCoefficients ());
    cvtPlaneCoeff2Vect(*ceil_coefficients, ceil_vect);
    cvtPlaneCoeff2Vect(*wall_coefficients, wall_vect);
    //intersection_line.resize(6);
    pcl::planeWithPlaneIntersection(ceil_vect, wall_vect, intersection_line, 0.1);
    cvtLineVect2Coeff(intersection_line, *line_coeff); //for visualization
//            std::cerr << "Line Coeff:" << std::endl << *line_coeff << std::endl;
    edge_coeff = *line_coeff;

    /*Compute the sqr distance between a point and a line*/
    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices());
    Eigen::Vector4f point;
    double sqr_dis;
    for (size_t i = 0; i < wall_cloud.points.size(); i++){//wall.
        point.x() = wall_cloud.points[i].x;//wall.
        point.y() = wall_cloud.points[i].y;//wall.
        point.z() = wall_cloud.points[i].z;//wall.
        point.w() = 0;

        Eigen::Vector4f line_pt, line_dir;
        line_pt.x() = line_coeff->values[0];
        line_pt.y() = line_coeff->values[1];
        line_pt.z() = line_coeff->values[2];
        line_pt.w() = 0;
        line_dir.x() = line_coeff->values[3];
        line_dir.y() = line_coeff->values[4];
        line_dir.z() = line_coeff->values[5];
        line_dir.w() = 0;

        sqr_dis = pcl::sqrPointToLineDistance(point, line_pt, line_dir);

        if (sqr_dis <=0.01){
            line_inliers->indices.push_back(i);
        }
    }
    std::cout << "Line inliers size: " << line_inliers->indices.size() << std::endl;

    /*Point with line projection*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_clouds (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::ProjectInliers<pcl::PointXYZ> line_proj;
    line_proj.setModelType (pcl::SACMODEL_LINE);
    line_proj.setInputCloud (wall_cloud.makeShared());//wall.makeShared()
    line_proj.setModelCoefficients (line_coeff);
    line_proj.setIndices(line_inliers);
    line_proj.filter (*line_clouds);
    std::cerr << "Number of points projected onto the line: " << line_clouds ->points.size() << std::endl;

    /*Identify the endpoints of the line*/
    pcl::PointXYZ min_endpt, max_endpt;
    lineEndPointCompute(*line_clouds, 'y', min_endpt, max_endpt);
    edge_end1 = max_endpt;
    edge_end2 = min_endpt;

    /*Painting path position*/
    pcl::PointXYZ min_pt, max_pt;
    min_pt.x = min_endpt.x - paint_dir[0] * edge_paint_dis_;
    min_pt.y = min_endpt.y - paint_dir[1] * edge_paint_dis_;
    min_pt.z = min_endpt.z - paint_dir[2] * edge_paint_dis_;
    max_pt.x = max_endpt.x - paint_dir[0] * edge_paint_dis_;
    max_pt.y = max_endpt.y - paint_dir[1] * edge_paint_dis_;
    max_pt.z = max_endpt.z - paint_dir[2] * edge_paint_dis_;
    std::cout << "Start point: " << max_pt << std::endl;
    std::cout << "End point: " << min_pt << std::endl;
    start_point = max_pt;
    end_point = min_pt;

    waypoints.push_back(max_pt);
    waypoints.push_back(min_pt);
    trace_vector << min_pt.x-max_pt.x, min_pt.y - max_pt.y, min_pt.z-max_pt.z;
    trace_vector.normalize();

    //Remove the edge and update the wall
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(wall_cloud.makeShared());
    pass.setFilterFieldName("z");
    if (type.compare("ceiling") == 0){
        float cut_height = (max_endpt.z + min_endpt.z) / 2 - edge_wall_cut_dis_;
        pass.setFilterLimits(-5, cut_height);
    } else if (type.compare("floor") == 0){
        float cut_height = (max_endpt.z + min_endpt.z) / 2 + edge_wall_cut_dis_;
        pass.setFilterLimits(cut_height, 5);
    }
    pass.filter(wall_cloud_updated);
}

void CeilPath::planeSeg(const pcl::PointCloud<pcl::PointXYZ> cloud, const float thresh, pcl::ModelCoefficients &coeffs, pcl::PointIndices &inliers ){
    /*function to identify the plane model for the given point cloud */
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (thresh);

    seg.setInputCloud (cloud.makeShared());
    seg.segment (inliers, coeffs);

    std::cerr << "PointCloud after segmentation has: "
              << inliers.indices.size () << " inliers." << std::endl;
}

void CeilPath::cvtPlaneCoeff2Vect(pcl::ModelCoefficients coeffs, Eigen::Vector4f &vector){
    vector.x() = coeffs.values[0];
    vector.y() = coeffs.values[1];
    vector.z() = coeffs.values[2];
    vector.w() = coeffs.values[3];
}

void CeilPath::cvtLineVect2Coeff(Eigen::VectorXf vector, pcl::ModelCoefficients &coeffs){
    //coeffs.values.resize(6);
    for(size_t i = 0; i < 6; i++){
        coeffs.values.push_back(vector[i]);
    }
}

void CeilPath::lineEndPointCompute(pcl::PointCloud<pcl::PointXYZ> cloud, char field_name, pcl::PointXYZ &minp, pcl::PointXYZ &maxp){
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    switch (field_name) {
        case 'x' :
            for (size_t i = 0; i < cloud.points.size(); i++){
                if (cloud.points[i].x == min_pt.x()){
                    minp = cloud.points[i];
                } else if (cloud.points[i].x == max_pt.x()){
                    maxp = cloud.points[i];
                } else {
                    continue;
                }
            }
            break;
        case 'y' :
            for (size_t i = 0; i < cloud.points.size(); i++){
                if (cloud.points[i].y == min_pt.y()){
                    minp = cloud.points[i];
                } else if (cloud.points[i].y == max_pt.y()){
                    maxp = cloud.points[i];
                } else {
                    continue;
                }
            }
            break;
        case 'z' :
            for (size_t i = 0; i < cloud.points.size(); i++){
                if (cloud.points[i].z == min_pt.z()){
                    minp = cloud.points[i];
                } else if (cloud.points[i].z == max_pt.z()){
                    maxp = cloud.points[i];
                } else {
                    continue;
                }
            }
            break;
        default :
            std::cerr << "Invalid field names!" << std::endl;

    }
}

