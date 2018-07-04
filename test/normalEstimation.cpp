
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

double input_d = 0.4;
// visualizer
pcl::visualization::PCLVisualizer viewer ("Segmention Visualization");


// get average vector of plane normal
int getAverageVector(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int idx){
  int step = 10;
  int count =0;
  for (size_t i = 0; i < normals->points.size (); i += step){
    planes_avgNormals->points[idx].normal_x += normals->points[i].normal_x;
    planes_avgNormals->points[idx].normal_y += normals->points[i].normal_y;
    planes_avgNormals->points[idx].normal_z += normals->points[i].normal_z;
    count++;
  } 
  planes_avgNormals->points[idx].normal_x = planes_avgNormals->points[idx].normal_x/count;
  planes_avgNormals->points[idx].normal_y = planes_avgNormals->points[idx].normal_y/count;
  planes_avgNormals->points[idx].normal_z = planes_avgNormals->points[idx].normal_z/count;
} 




// 0: others, 1: wall, 2: ceiling, 3: floor
int getPlaneState(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud, pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals, int index){

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (plane_cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets with init normals var
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (input_d);   // or ne.setKSearch();
  // Compute the features
  ne.compute (*cloud_normals);

  //find all plane normal vector and output
  getAverageVector(cloud_normals, planes_avgNormals, 0);
  pcl::PCDWriter writer;
  writer.write<pcl::Normal> ("normal_vector.pcd", *cloud_normals, false);
  writer.write<pcl::Normal> ("plane_normal_vector.pcd", *planes_avgNormals, false);
  std::cout << "\n - [MAIN] Avg normal Vector  " << planes_avgNormals->points[0] << std::endl;

  //update viewer
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (plane_cloud, cloud_normals, 10, 0.28, "normals");
  viewer.addCoordinateSystem (1.0);

  // return appropriate state
  float z_vector = planes_avgNormals->points[index].normal_z;
  if (z_vector > 0.92){ //floor
    return 3;
  }
  else if (z_vector < -0.92){ //ceiling
    return 2;
  }
  else if (z_vector < 0.08 && z_vector > -0.08){
    return 1;
  }
  else{ //others
    return 0;
  }
}



int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> filenames;

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () != 1) 
      return -1;
  }
  if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    return -1;
  }

    // find double agument in command line
  if (pcl::console::find_switch (argc, argv, "-d")){
    int input_idx2 = pcl::console::find_argument (argc, argv, "-d") + 1;
    std::stringstream ss( argv[input_idx2] );

    if ( !(ss >> input_d))
      std::cout << "Invalid double...\n";
    std::cout << "Input Double arg for '-d' is " << input_d << std::endl << std::endl;
  } 

  // get plane normal then get state
  pcl::PointCloud<pcl::Normal>::Ptr planes_avgNormals (new pcl::PointCloud<pcl::Normal>);
  planes_avgNormals->width = 4; // size!!!!! number of planes in model
  planes_avgNormals->height = 1;
  planes_avgNormals->is_dense = true;
  planes_avgNormals->points.resize (planes_avgNormals->width * planes_avgNormals->height) ;
  int plane_index = 0; //temp
  int state = getPlaneState( cloud, planes_avgNormals, plane_index );
  std::cout << " [state] " << state << std::endl;

  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 23, 180, 180);
  viewer.addPointCloud (cloud, cloud_color_handler, "cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");


  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }


}
