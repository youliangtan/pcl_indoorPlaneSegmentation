# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pictobot/pcl_indoorPlaneSegmentation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pictobot/pcl_indoorPlaneSegmentation/build

# Include any dependencies generated for this target.
include CMakeFiles/indoor_planar_segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/indoor_planar_segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/indoor_planar_segmentation.dir/flags.make

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o: CMakeFiles/indoor_planar_segmentation.dir/flags.make
CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o: ../indoor_planar_segmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o -c /home/pictobot/pcl_indoorPlaneSegmentation/indoor_planar_segmentation.cpp

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pictobot/pcl_indoorPlaneSegmentation/indoor_planar_segmentation.cpp > CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.i

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pictobot/pcl_indoorPlaneSegmentation/indoor_planar_segmentation.cpp -o CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.s

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.requires:

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.requires

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.provides: CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/indoor_planar_segmentation.dir/build.make CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.provides

CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.provides.build: CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o


CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o: CMakeFiles/indoor_planar_segmentation.dir/flags.make
CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o: ../pointcloudSegmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o -c /home/pictobot/pcl_indoorPlaneSegmentation/pointcloudSegmentation.cpp

CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pictobot/pcl_indoorPlaneSegmentation/pointcloudSegmentation.cpp > CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.i

CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pictobot/pcl_indoorPlaneSegmentation/pointcloudSegmentation.cpp -o CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.s

CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.requires:

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.requires

CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.provides: CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/indoor_planar_segmentation.dir/build.make CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.provides.build
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.provides

CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.provides.build: CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o


CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o: CMakeFiles/indoor_planar_segmentation.dir/flags.make
CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o: ../planeSegmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o -c /home/pictobot/pcl_indoorPlaneSegmentation/planeSegmentation.cpp

CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pictobot/pcl_indoorPlaneSegmentation/planeSegmentation.cpp > CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.i

CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pictobot/pcl_indoorPlaneSegmentation/planeSegmentation.cpp -o CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.s

CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.requires:

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.requires

CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.provides: CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/indoor_planar_segmentation.dir/build.make CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.provides.build
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.provides

CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.provides.build: CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o


CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o: CMakeFiles/indoor_planar_segmentation.dir/flags.make
CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o: ../patchSegmentation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o -c /home/pictobot/pcl_indoorPlaneSegmentation/patchSegmentation.cpp

CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pictobot/pcl_indoorPlaneSegmentation/patchSegmentation.cpp > CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.i

CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pictobot/pcl_indoorPlaneSegmentation/patchSegmentation.cpp -o CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.s

CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.requires:

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.requires

CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.provides: CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/indoor_planar_segmentation.dir/build.make CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.provides.build
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.provides

CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.provides.build: CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o


# Object files for target indoor_planar_segmentation
indoor_planar_segmentation_OBJECTS = \
"CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o" \
"CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o" \
"CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o" \
"CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o"

# External object files for target indoor_planar_segmentation
indoor_planar_segmentation_EXTERNAL_OBJECTS =

indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o
indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o
indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o
indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o
indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/build.make
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_regex.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
indoor_planar_segmentation: /usr/local/lib/libpcl_common.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
indoor_planar_segmentation: /usr/local/lib/libpcl_kdtree.so
indoor_planar_segmentation: /usr/local/lib/libpcl_octree.so
indoor_planar_segmentation: /usr/local/lib/libpcl_search.so
indoor_planar_segmentation: /usr/local/lib/libpcl_sample_consensus.so
indoor_planar_segmentation: /usr/local/lib/libpcl_filters.so
indoor_planar_segmentation: /usr/local/lib/libpcl_tracking.so
indoor_planar_segmentation: /usr/lib/libOpenNI.so
indoor_planar_segmentation: /usr/local/lib/libpcl_io.so
indoor_planar_segmentation: /usr/local/lib/libpcl_features.so
indoor_planar_segmentation: /usr/local/lib/libpcl_visualization.so
indoor_planar_segmentation: /usr/local/lib/libpcl_ml.so
indoor_planar_segmentation: /usr/local/lib/libpcl_segmentation.so
indoor_planar_segmentation: /usr/local/lib/libpcl_people.so
indoor_planar_segmentation: /usr/local/lib/libpcl_registration.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
indoor_planar_segmentation: /usr/local/lib/libpcl_surface.so
indoor_planar_segmentation: /usr/local/lib/libpcl_recognition.so
indoor_planar_segmentation: /usr/local/lib/libpcl_keypoints.so
indoor_planar_segmentation: /usr/local/lib/libpcl_stereo.so
indoor_planar_segmentation: /usr/local/lib/libpcl_outofcore.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_system.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libboost_regex.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
indoor_planar_segmentation: /usr/lib/libOpenNI.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOAMR-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOInfovis-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtklibxml2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingMath-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOImport-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOSQL-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOVideo-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOExodus-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkChartsCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingImage-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOMINC-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOPLY-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOMovie-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkoggtheora-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkGeovisCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOExport-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkgl2ps-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOEnSight-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkInteractionImage-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOParallel-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingStencil-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libpcl_common.so
indoor_planar_segmentation: /usr/local/lib/libpcl_kdtree.so
indoor_planar_segmentation: /usr/local/lib/libpcl_octree.so
indoor_planar_segmentation: /usr/local/lib/libpcl_search.so
indoor_planar_segmentation: /usr/local/lib/libpcl_sample_consensus.so
indoor_planar_segmentation: /usr/local/lib/libpcl_filters.so
indoor_planar_segmentation: /usr/local/lib/libpcl_tracking.so
indoor_planar_segmentation: /usr/local/lib/libpcl_io.so
indoor_planar_segmentation: /usr/local/lib/libpcl_features.so
indoor_planar_segmentation: /usr/local/lib/libpcl_visualization.so
indoor_planar_segmentation: /usr/local/lib/libpcl_ml.so
indoor_planar_segmentation: /usr/local/lib/libpcl_segmentation.so
indoor_planar_segmentation: /usr/local/lib/libpcl_people.so
indoor_planar_segmentation: /usr/local/lib/libpcl_registration.so
indoor_planar_segmentation: /usr/local/lib/libpcl_surface.so
indoor_planar_segmentation: /usr/local/lib/libpcl_recognition.so
indoor_planar_segmentation: /usr/local/lib/libpcl_keypoints.so
indoor_planar_segmentation: /usr/local/lib/libpcl_stereo.so
indoor_planar_segmentation: /usr/local/lib/libpcl_outofcore.so
indoor_planar_segmentation: /usr/local/lib/libvtksqlite-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkInfovisCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkproj4-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkverdict-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkViewsCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOXML-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingSources-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingColor-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkfreetype-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkjsoncpp-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIONetCDF-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkexoIIc-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkNetCDF-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkhdf5-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOGeometry-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkParallelCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingFourier-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkalglib-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOImage-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkDICOMParser-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkmetaio-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkpng-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtktiff-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkjpeg-7.1.so.1
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libm.so
indoor_planar_segmentation: /usr/local/lib/libvtkglew-7.1.so.1
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libSM.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libICE.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libX11.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libXext.so
indoor_planar_segmentation: /usr/lib/x86_64-linux-gnu/libXt.so
indoor_planar_segmentation: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkRenderingCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonColor-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersSources-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkFiltersCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOLegacy-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkexpat-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkIOCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkzlib-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkImagingCore-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonMisc-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonSystem-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtksys-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonMath-7.1.so.1
indoor_planar_segmentation: /usr/local/lib/libvtkCommonCore-7.1.so.1
indoor_planar_segmentation: CMakeFiles/indoor_planar_segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable indoor_planar_segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/indoor_planar_segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/indoor_planar_segmentation.dir/build: indoor_planar_segmentation

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/build

CMakeFiles/indoor_planar_segmentation.dir/requires: CMakeFiles/indoor_planar_segmentation.dir/indoor_planar_segmentation.cpp.o.requires
CMakeFiles/indoor_planar_segmentation.dir/requires: CMakeFiles/indoor_planar_segmentation.dir/pointcloudSegmentation.cpp.o.requires
CMakeFiles/indoor_planar_segmentation.dir/requires: CMakeFiles/indoor_planar_segmentation.dir/planeSegmentation.cpp.o.requires
CMakeFiles/indoor_planar_segmentation.dir/requires: CMakeFiles/indoor_planar_segmentation.dir/patchSegmentation.cpp.o.requires

.PHONY : CMakeFiles/indoor_planar_segmentation.dir/requires

CMakeFiles/indoor_planar_segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/indoor_planar_segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/clean

CMakeFiles/indoor_planar_segmentation.dir/depend:
	cd /home/pictobot/pcl_indoorPlaneSegmentation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pictobot/pcl_indoorPlaneSegmentation /home/pictobot/pcl_indoorPlaneSegmentation /home/pictobot/pcl_indoorPlaneSegmentation/build /home/pictobot/pcl_indoorPlaneSegmentation/build /home/pictobot/pcl_indoorPlaneSegmentation/build/CMakeFiles/indoor_planar_segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/indoor_planar_segmentation.dir/depend
