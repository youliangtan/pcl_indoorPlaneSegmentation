/*
*Create By Tan You Liang, Under Transforma Project
*Jul 2018, CopyRight
*Acknowledge before using this code
*/

#include "visualizer.h"


PCVisualizer::PCVisualizer( ){
    viewer.setWindowName ( "Segmention Visualization" );
    v1 = 1; // viewer port num
    v2 = 2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1); // viewer 1
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2); //viewer 2  
    viewer.setBackgroundColor(0.05, 0.05, 0.05, v1);
    viewer.addCoordinateSystem (1.0, "v1_transformPointCloud", v1); 
    viewer.addCoordinateSystem (1.0, "v2_axis", v2);
    viewer.setBackgroundColor(0.2, 0.2, 0.2, v2);
}

// view cloud as white for 2 viewers
void PCVisualizer::addSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud ){
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (source_cloud, 255, 255, 255);
    viewer.addPointCloud (source_cloud, cloud_color_handler, "source_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
    viewer.addText("Pointcloud Plane Segmentation", 100, 10, "=')");
}

// visualizer single patch in viewer 2
void PCVisualizer::addPatch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPatch, int index ){
    std::cout << "cloud patches size: "<< cloudPatch->points.size () << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> patch_color_handler (cloudPatch, 23 + index*80, 200 , 223- index*70);
    viewer.addPointCloud (cloudPatch, patch_color_handler, "filtered_patch_" + std::to_string(index), v1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "filtered_patch_" + std::to_string(index));
}

//output vector of PlaneStruct to viewer 2 
void PCVisualizer::addPlaneStruct( std::vector < PlaneStruct > *cloudPlanes ){
    
    std::cout << "\n - Running Visualizer -" << std::endl;
    for (size_t i= 0 ; i< cloudPlanes->size() ; i++ ){

    // add point cloud to visualizer scene
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color_handler ( (*cloudPlanes)[i].cloud, 
        180*(i+1) , i*88 - 22 , (i+1)*53); //plane colour
    viewer.addPointCloud ( (*cloudPlanes)[i].cloud, cluster_color_handler, "planar_cloud" + std::to_string(i), v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, 
        "planar_cloud" + std::to_string(i));  
    viewer.addText3D( (*cloudPlanes)[i].type + std::to_string(i) , (*cloudPlanes)[i].cloud->points[ (*cloudPlanes)[i].cloud->points.size() / 2 ], 
        0.12, 0.0, 1.0, 0.0, "cluster_text"+ std::to_string(i));
    std::cout << (*cloudPlanes)[i].type << i << " with plane size: " << (*cloudPlanes)[i].cloud->points.size() << std::endl;
    
    }
}

// add waypoints to viewer 1
//void PCVisualizer::addWayPoints( std::vector<pcl::PointXYZ> whole_waypoints, std::vector<pcl::PointXYZ> whole_wallpoints){

//    for (size_t i = 0; i < whole_waypoints.size(); i++){
//        std::ostringstream corner_name;
//        corner_name << "waypoint_" << i;
//        viewer.addSphere(whole_waypoints[i], 0.03, 0, 255, 0, corner_name.str());
//    }
//    for (size_t i = 0; i < whole_wallpoints.size(); i++){
//        std::ostringstream corner_name;
//        corner_name << "wallpoint_" << i;
//        viewer.addSphere(whole_wallpoints[i], 0.03, 0, 0, 255, corner_name.str());
//    }
//    for (size_t i = 0; i < whole_wallpoints.size(); i++){
//        std::ostringstream arrow_name;
//        arrow_name << "arrow_" << i;
//        viewer.addArrow(whole_wallpoints[i], whole_waypoints[i], 0, 0, 155, false, arrow_name.str());
//    }
//}
void PCVisualizer::addWayPoints( std::vector<pcl::PointXYZ> whole_waypoints, std::vector<pcl::PointXYZ> whole_wallpoints,
                                 std::vector<std::vector<PathTrace> > path_traces){
    for (size_t i = 0; i < path_traces.size(); i++){
        for (size_t j = 0; j < path_traces[i].size(); j++){
            std::ostringstream start_name;
            start_name << "p" << i << "start" << j;
            viewer.addSphere(cvtEigen2PclXYZ(path_traces[i][j].start_point), 0.03, 0, 0, 255, start_name.str());

            std::ostringstream end_name;
            end_name << "p" << i << "end" << j;
            viewer.addSphere(cvtEigen2PclXYZ(path_traces[i][j].end_point), 0.03, 255, 0, 0, end_name.str());

            std::ostringstream path_name;
            path_name << "p" << i << "path" << j;
            viewer.addArrow(cvtEigen2PclXYZ(path_traces[i][j].end_point), cvtEigen2PclXYZ(path_traces[i][j].start_point), 0, 155, 155, false, path_name.str());
        }
    }

    for (size_t i = 0; i < whole_wallpoints.size(); i++){
        std::ostringstream arrow_name;
        arrow_name << "arrow_" << i;
        viewer.addArrow(whole_wallpoints[i], whole_waypoints[i], 0, 0, 155, false, arrow_name.str());
    }
}

// run viewer to visualize previous added pointclouds
void PCVisualizer::runViewer(){

    viewer.setPosition(800, 400); // Setting visualiser window position
    viewer.setCameraPosition(-5,-5,5,0,0,1);
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
    
}

pcl::PointXYZ PCVisualizer::cvtEigen2PclXYZ(Eigen::Vector3f vector){
    pcl::PointXYZ point;
    point.x = vector[0];
    point.y = vector[1];
    point.z = vector[2];
    return point;
}


