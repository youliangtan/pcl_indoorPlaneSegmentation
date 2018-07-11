# pcl_indoorPlaneSegmentation
This PCL perception tool is used to identify target plane for a semi autonomous painting robot. Potential planes within working distance will be identified, then further classifided to their type. The pointcloud will then be stored in a vector of struct for further used. A visualizer is used to visualized the segmentation result. 

Make by `cd .../build`, then `make -j4`

To run the exe files:
`./indoor_planar_segmentation "pcd_path.pcd"`
To purely visualize Point clouds
`./pc_visualizer "pcd_path1.pcd" "optional_pcd_path2.pcd"`

### Example 1
![alt text](/documentation/example1.png?)


### Example 2
![alt text](/documentation/example2.png?)


### pcd files
ALl pcd sample files are located in the `.../pcd` folder. Corresponded sample images with the pcd files are shown in the same folder.

###ROS service packag
Please Refer to `.../ros_pkg_srv` folder.
