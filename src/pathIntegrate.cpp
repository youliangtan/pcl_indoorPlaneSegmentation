#include "paintPathPlanning.h"

void pathIntegrate(std::vector<PlaneStruct> input_patches, int patch_size, std::vector<std::vector<PathTrace> > &pathTraces){

    pcl::PointCloud<pcl::PointXYZ>::Ptr whole_cloud (new pcl::PointCloud<pcl::PointXYZ> ()); //visualization purpose
    std::vector<pcl::PointXYZ> whole_waypoints; //visualization purpose
    std::vector<pcl::PointXYZ> whole_wallpoints;    //visualization purpose

    for (size_t i = 0; i < input_patches.size(); i++){
        *whole_cloud += *(input_patches[i].cloud);
    }

    for(size_t j = 0; j < patch_size; j++){

        std::vector<int> wall_indices;
        std::vector<int> ceil_indices;
        std::vector<int> floor_indices;

        for(size_t i = 0; i < input_patches.size(); i++){
            if (input_patches[i].patchNum == j){
                // std::cout << "TYPE!!!>> " << input_patches[i].type  << std::endl;
                //types.push_back(input_patches[i].type);
                if (input_patches[i].type.compare("wall") == 0){
                    wall_indices.push_back(i);
                } else if (input_patches[i].type.compare("ceiling") == 0){
                    ceil_indices.push_back(i);
                } else if (input_patches[i].type.compare("floor") == 0){
                    floor_indices.push_back(i);
                } else{
                    std::cerr << "Unexpected wall types. It is to be developed in the future!"
                              << "Type: " << input_patches[i].type << std::endl;
                    return;
                }
            }
        }

        std::cout << "Indices: " << wall_indices.size() << " " << ceil_indices.size() << " " << floor_indices.size() << std::endl;
        if (wall_indices.size() == 1 && ceil_indices.size() == 0 && floor_indices.size() == 0){
            WallPathPlan wallPathPlan(*(input_patches[wall_indices[0]].cloud));
            wallPathPlan.getWaypoints();

            //save the waypoints
            PathTrace path;
            std::vector<PathTrace> paths;
            for (size_t i = 0; i < wallPathPlan.waypoints.size(); i += 2){
                path.start_point << wallPathPlan.waypoints[i].x, wallPathPlan.waypoints[i].y, wallPathPlan.waypoints[i].z;
                path.end_point << wallPathPlan.waypoints[i+1].x, wallPathPlan.waypoints[i+1].y, wallPathPlan.waypoints[i+1].z;
                path.normal_vector = wallPathPlan.plane_normal;
                path.trace_vector = wallPathPlan.trace_vector[i/2];
                paths.push_back(path);
            }
            pathTraces.push_back(paths);

            whole_waypoints.insert(whole_waypoints.end(), wallPathPlan.waypoints.begin(), wallPathPlan.waypoints.end()); //visualization
            whole_wallpoints.insert(whole_wallpoints.end(), wallPathPlan.wallpoints.begin(), wallPathPlan.wallpoints.end()); //visual

        } else if (wall_indices.size() == 1 && ceil_indices.size() == 1 && floor_indices.size() == 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cut (new pcl::PointCloud<pcl::PointXYZ> ());
            CeilPath ceilPath(*(input_patches[wall_indices[0]].cloud), *(input_patches[ceil_indices[0]].cloud), "ceiling");
            ceilPath.getWaypoints();
            *wall_cut = ceilPath.wall_cloud_updated;
            WallPathPlan wallPathPlan(*wall_cut);
            wallPathPlan.getWaypoints();

            //save the waypoints
            PathTrace path;
            std::vector<PathTrace> paths;
            for (size_t i = 0; i < ceilPath.waypoints.size(); i += 2){ //edge waypoints
                path.start_point << ceilPath.waypoints[i].x, ceilPath.waypoints[i].y, ceilPath.waypoints[i].z;
                path.end_point << ceilPath.waypoints[i+1].x, ceilPath.waypoints[i+1].y, ceilPath.waypoints[i+1].z;
                path.normal_vector = ceilPath.paint_dir;
                path.trace_vector = ceilPath.trace_vector;
                paths.push_back(path);
            }

            for (size_t i = 0; i < wallPathPlan.waypoints.size(); i += 2){//wall waypoints
                path.start_point << wallPathPlan.waypoints[i].x, wallPathPlan.waypoints[i].y, wallPathPlan.waypoints[i].z;
                path.end_point << wallPathPlan.waypoints[i+1].x, wallPathPlan.waypoints[i+1].y, wallPathPlan.waypoints[i+1].z;
                path.normal_vector = wallPathPlan.plane_normal;
                path.trace_vector = wallPathPlan.trace_vector[i/2];
                paths.push_back(path);
            }
            pathTraces.push_back(paths);

            //visualization
            whole_waypoints.insert(whole_waypoints.end(), ceilPath.waypoints.begin(), ceilPath.waypoints.end());
            whole_waypoints.insert(whole_waypoints.end(), wallPathPlan.waypoints.begin(), wallPathPlan.waypoints.end());
            whole_wallpoints.push_back(ceilPath.edge_end1);     whole_wallpoints.push_back(ceilPath.edge_end2);
            whole_wallpoints.insert(whole_wallpoints.end(), wallPathPlan.wallpoints.begin(), wallPathPlan.wallpoints.end());


        } else if (wall_indices.size() == 1 && ceil_indices.size() == 0 && floor_indices.size() == 1){
            pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cut (new pcl::PointCloud<pcl::PointXYZ> ());
            CeilPath ceilPath(*(input_patches[wall_indices[0]].cloud), *(input_patches[floor_indices[0]].cloud), "floor");
            ceilPath.getWaypoints();
            *wall_cut = ceilPath.wall_cloud_updated;
            WallPathPlan wallPathPlan(*wall_cut);
            wallPathPlan.getWaypoints();

            //save the waypoints
            PathTrace path;
            std::vector<PathTrace> paths;
            for (size_t i = 0; i < ceilPath.waypoints.size(); i += 2){ //edge waypoints
                path.start_point << ceilPath.waypoints[i].x, ceilPath.waypoints[i].y, ceilPath.waypoints[i].z;
                path.end_point << ceilPath.waypoints[i+1].x, ceilPath.waypoints[i+1].y, ceilPath.waypoints[i+1].z;
                path.normal_vector = ceilPath.paint_dir;
                path.trace_vector = ceilPath.trace_vector;
                paths.push_back(path);
            }

            for (size_t i = 0; i < wallPathPlan.waypoints.size(); i += 2){//wall waypoints
                path.start_point << wallPathPlan.waypoints[i].x, wallPathPlan.waypoints[i].y, wallPathPlan.waypoints[i].z;
                path.end_point << wallPathPlan.waypoints[i+1].x, wallPathPlan.waypoints[i+1].y, wallPathPlan.waypoints[i+1].z;
                path.normal_vector = wallPathPlan.plane_normal;
                path.trace_vector = wallPathPlan.trace_vector[i/2];
                paths.push_back(path);
            }
            pathTraces.push_back(paths);

            //visualization
            whole_waypoints.insert(whole_waypoints.end(), wallPathPlan.waypoints.begin(), wallPathPlan.waypoints.end());
            whole_waypoints.insert(whole_waypoints.end(), ceilPath.waypoints.begin(), ceilPath.waypoints.end());
            //whole_wallpoints.push_back(ceilPath.edge_end1);     whole_wallpoints.push_back(ceilPath.edge_end2);
            whole_wallpoints.insert(whole_wallpoints.end(), wallPathPlan.wallpoints.begin(), wallPathPlan.wallpoints.end());
            whole_wallpoints.push_back(ceilPath.edge_end1);     whole_wallpoints.push_back(ceilPath.edge_end2);
        } else {
            std::cerr << "Fail to generate the waypoints!" << std::endl;
        }
        std::cerr<< std::endl << std::endl << std::endl;
    }

    std::cerr << "Num of ways: " << pathTraces.size() << std::endl;

    pcl::visualization::PCLVisualizer viewer ("Patch viewing");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    viewer.addPointCloud<pcl::PointXYZ> (whole_cloud, "whole_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "whole_cloud");

    for (size_t i = 0; i < whole_waypoints.size(); i++){
        std::ostringstream corner_name;
        corner_name << "waypoint_" << i;
        viewer.addSphere(whole_waypoints[i], 0.03, 0, 255, 0, corner_name.str());
    }

    for (size_t i = 0; i < whole_wallpoints.size(); i++){
        std::ostringstream corner_name;
        corner_name << "wallpoint_" << i;
        viewer.addSphere(whole_wallpoints[i], 0.03, 0, 0, 255, corner_name.str());
    }

    for (size_t i = 0; i < whole_wallpoints.size(); i++){
        std::ostringstream arrow_name;
        arrow_name << "arrow_" << i;
        viewer.addArrow(whole_wallpoints[i], whole_waypoints[i], 0, 0, 155, false, arrow_name.str());
    }

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce(100);
    }

}
