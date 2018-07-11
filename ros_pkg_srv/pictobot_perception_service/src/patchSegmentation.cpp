
#include "patchSegmentation.h"

// set boundary height of-axis of ceilign and floor according to threshold in config
class BoundaryHeight{

  private:
    double ceiling, floor;

  public:
    BoundaryHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
    
      int index;
      double floor_thresh = configParam["Boundary"]["z_Thresh"][0].asDouble();
      double ceiling_thresh = configParam["Boundary"]["z_Thresh"][1].asDouble();
      std::list<double> z_data;
      std::list<double>::iterator it = z_data.begin();

      for (size_t i=0; i < source_cloud->points.size (); ++i){
        z_data.push_back( source_cloud->points[i].z );
      }
      z_data.sort(); //sort number
      
      index = z_data.size() * ceiling_thresh;
      std::advance(it, index);
      ceiling = *it;
      it = z_data.begin();

      index = z_data.size() * floor_thresh;
      std::advance(it, index);
      floor = *it;
    }

    double Ceiling() { return ceiling; }
    double Floor() { return floor; }
};


// Set Point cloud boundary according to config param
pcl::PointCloud<pcl::PointXYZ>::Ptr PC2Patches::setPCBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){ // filtered_cloud){
  
  // build the condition
  cloud_filtered = (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >) new pcl::PointCloud<pcl::PointXYZ>() ;

  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
  BoundaryHeight boundaryHeight(source_cloud);

  // x-axis
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, configParam["Boundary"]["x"][0].asDouble()  )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, configParam["Boundary"]["x"][1].asDouble()  )));

  // y-axis
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, configParam["Boundary"]["y"][0].asDouble()  )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, configParam["Boundary"]["y"][1].asDouble()  )));

  // z-axis
  floorHeight = boundaryHeight.Floor();
  ceilingHeight = boundaryHeight.Ceiling();
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, floorHeight  )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, ceilingHeight  )));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (source_cloud);
  condrem.setKeepOrganized(true);
  
  // apply filter
  condrem.filter (*cloud_filtered);
  return cloud_filtered;
} 


//split filtered point cloud to several horizonal patches
pcl::PointCloud<pcl::PointXYZ>::Ptr PC2Patches::getPatch(int patchNum){
  double higher_bound, lower_bound;
  pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

  higher_bound = ceilingHeight - patchHeight*patchNum;
  lower_bound = ceilingHeight - patchHeight*(patchNum + 1);
  if (higher_bound < floorHeight){
    std::cerr << "Error! Patching for " << patchNum << " out of bound!!!" << std::endl;
    return patch_cloud;
  }
  if (lower_bound < floorHeight) { lower_bound = floorHeight; }

  // z-axis filtering
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, lower_bound  )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, higher_bound  )));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_filtered);
  condrem.setKeepOrganized(true);
  condrem.filter (*patch_cloud);  // apply filter

  //remove nan in pointcloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud( *patch_cloud, *patch_cloud, indices );

  std::cout << "PatchNum " << patchNum << " >> " << lower_bound  << "\t" << higher_bound << std::endl; 
  return patch_cloud;
}


//get exception patches between each patch
pcl::PointCloud<pcl::PointXYZ>::Ptr PC2Patches::getExceptionPatch(int upper_patchNum){

  double higher_bound, lower_bound;
  pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
  double height_between_patches = ceilingHeight - patchHeight*upper_patchNum;
  
  higher_bound = height_between_patches + 0.05;
  lower_bound = height_between_patches - 0.05;

  // z-axis filtering
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, lower_bound  )));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr 
    (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, higher_bound  )));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_filtered);
  condrem.setKeepOrganized(true);
  condrem.filter (*patch_cloud);  // apply filter

  //remove nan in pointcloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud( *patch_cloud, *patch_cloud, indices );
  return patch_cloud;
}


//get number of patches in filtered point cloud
int PC2Patches::getNumberOfPatches(){
  double totalHeight = ceilingHeight - floorHeight;
  patchHeight = configParam["Patch"]["height"].asDouble();
  std::cout << "Floor to Ceiling: " << totalHeight << std::endl;
  return ceil(totalHeight/patchHeight);
}