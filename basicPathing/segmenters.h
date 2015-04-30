#include "fc2triclops.h"
#include "triclops.h"
#include <boost/thread/thread.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

enum REGULAR_OPERATIONS {
	CENTER= 0, LEFT =1, RIGHT, NONE, VISION, GPS
};

struct ImageContainer
{
	FC2::Image unprocessed[2];
	FC2::Image bgru[2];
	FC2::Image mono[2];
	FC2::Image packed;
};
	// Container of Images used for processing
	ImageContainer imageContainer;

struct Robot;
int turn(int degrees);
int at_goal(Robot* robot);
int gps_target(cloudPtr cloud, Robot* robot);
void compute_step(cloudPtr, Robot* robot);
void test_view(cloudPtr cloud);


int configureCamera( FC2::Camera &camera );
int generateTriclopsContext( FC2::Camera     & camera,
                             TriclopsContext & triclops );
int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );
int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );
int generateTriclopsInput( FC2::Image const & grabbedImage,
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData );
int doStereo( TriclopsContext const & triclops,
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage );

pcl::PointCloud<pcl::PointXYZRGB>::Ptr save3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData,
                  std::string outname );


//
// Macro to check, report on, and handle Triclops API error codes.
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

struct Robot{

	public:
		//static Robot * instance;
		Robot(){
			//instance = new Robot();
			method=GPS;
			checkDirection=CENTER;
			checking=0;
			check_result=0;
			checkDirection=CENTER;
			goal_num=0;
		}
		Robot(const Robot & source){}//disabling copy constructor
		//Robot_singleton(const Robot_singleton && source){}//disabling move constructor
		//Robot_singleton(Robot_singleton const &)= delete;
		//Robot_singleton(Robot_singleton&&)= delete;

		int method;
		int checkDirection;
		int checking;
		int check_result;
		float goalBearing;
		float goalDistance;
		float prevGoalDistance;
		float bearing;
		float latitude;
		float longitude;
		int goal_num;
		double goal_lat[10];
		double goal_long [10];
		int Map[200*100*10];

		//Robot * getInstance() {
			//if (instance ==NULL){ }
			//return instance; }
};

void set_goals(Robot* robot){
	std::string  str;
	std::ifstream gpsGoals("gpsGoals.txt");

	if (gpsGoals.is_open()) {
		int goals=0;
		//get number of goals
		 do {
			std::getline(gpsGoals,str);
			goals++;
		}while (strcmp(str.c_str(), "#"));//do until # found
		 
		goals/=3;//3 lines per single entry
		robot->goal_num=goals;

		std::cout << "There are " << goals << " GPS goals" << std::endl;
		gpsGoals.clear();
		gpsGoals.seekg(0, gpsGoals.beg);
		for(int goal=0; goal < goals; goal++){
			getline(gpsGoals, str);
			getline(gpsGoals, str);
			str.erase(0, str.find(" ")+1);
			robot->goal_lat[goal]= atof(str.c_str());
			getline(gpsGoals, str);
			str.erase(0, str.find(" ")+1);
			robot->goal_long[goal]= atof(str.c_str());
		}
	}
	else{
		std::cout << "Error: No goals file exists" << std::endl;
	}
	/*debug
		for(int i=0; i<goals; i++){
			std::cout << "lat: " << goal_lat[i] << std::endl;
			std::cout << "lon: " << goal_long[i] << std::endl;
		}*/

}

float saveAnd_condense_object(Robot* robot, Cloud* inCloud){
	float long_diff, lat_diff;
	float dist_to_hazard,  bear_to_hazard;
	float x_ave=0, y_ave=0, z_ave=0;
	float x_sum=0, y_sum=0, z_sum=0;
	//return array
	float info_array[5];
	for(int i = 0; i < inCloud->points.size(); i++) {
		//sum the cloud 
		x_sum += inCloud->points[i].x;
		z_sum += inCloud->points[i].z;
		inCloud->points[i].y=0;
		//be aware, one may have overlapping points!!!!!!!!
	}

	//Average the cloud
	//Y is not included since the map will be flat
	x_ave = x_sum / inCloud->points.size();
	z_ave = z_sum / inCloud->points.size();

	std::cout << " x_ave "<< x_ave << std::endl;
	std::cout << " z_ave "<< z_ave << std::endl;

	//Determine absolute bearing of object
	float angular_diff = atan(x_ave/z_ave)*180/M_PI;//opposite/adj
 	if (x_ave > 0)
		bear_to_hazard= (robot->bearing+angular_diff);
		if (bear_to_hazard>360) bear_to_hazard-=360;
	else{
		bear_to_hazard= robot->bearing-angular_diff;
		if (bear_to_hazard<0) bear_to_hazard+=360;
	}
	dist_to_hazard= pow(x_ave*x_ave + z_ave*z_ave, 0.5);//Thanks Pythagoras

	//Determing GPS shift of object
		lat_diff = cos(bear_to_hazard)*dist_to_hazard;
		long_diff = sin(bear_to_hazard)*dist_to_hazard;
	//Determine coordinate of object
		
	info_array[0] = robot->latitude - lat_diff;
	info_array[1] = robot->longitude + long_diff;
	info_array[2] = dist_to_hazard; 
	info_array[3] = bear_to_hazard;
	return *info_array;

}

void update_gps(Robot* robot){
	//get singleton robot object to modify
	std::string  str;
	std::string command = "./client 192.168.1.102 8000 /home/pi/IGV/GPS/gpsData.txt";
		system(command.c_str());

	std::ifstream gpsData("gpsData.txt");

	double * m_position = new double [6];
	

	if (gpsData.is_open()) {
		for(int info=0; info < 5; info++){
			getline(gpsData, str);
			str.erase(0, str.find("=")+1);
			m_position[info] = atof(str.c_str());
		}
	}
	else{
		std::cout << "Error: No GPS file found!"  << std::endl;
	}
	
	///*Debug
		for(int i=0; i<5; i++){
			std::cout << "position Info " << m_position[i] << std::endl;
		} //*/
		double x_diff = m_position[1] - robot->goal_lat[robot->goal_num]; //difference is positive if goal is west
		double y_diff = robot->goal_long[robot->goal_num] - m_position[2]; //difference is positive if north

	//set bearing

		double correctBearing, Theta = atan2(y_diff, x_diff)*180/M_PI; 
		if (Theta <= 0){
			correctBearing = (Theta * -1) + 90;
		}
		else if (Theta > 90) {
			Theta -= 90;
			correctBearing = 360 - Theta;
		}
		else {//theta 
			correctBearing = 90-Theta;
		}
		robot->goalDistance=pow((pow(x_diff,2)+pow(y_diff,2)), 0.5 );
		robot->bearing = m_position[4];
		robot->goalBearing = correctBearing;
		robot->latitude = m_position[0];
		robot->longitude = m_position[1];
	return;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > noiseFilter(cloudPtr cloud) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.021);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.08); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > cluster_extract;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>* cloud_cluster =(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		cluster_extract.push_back(cloud_cluster);
  }
	return cluster_extract;
}
std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > regionSeg(cloudPtr cloud) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.005f, 0.005f, 0.005f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.001);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.08); // 2cm
  ec.setMinClusterSize (300);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > cluster_extract;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>* cloud_cluster =(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		cluster_extract.push_back(cloud_cluster);
  }
	return cluster_extract;
}
cloudPtr plainSeg(cloudPtr cloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return cloud;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  cloudPtr inCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < inliers->indices.size (); ++i)
  {
    pcl::PointXYZRGB point;
    point.x = cloud->points[inliers->indices[i]].x;
    point.y = cloud->points[inliers->indices[i]].y;
    point.z = cloud->points[inliers->indices[i]].z;
    point.r = cloud->points[inliers->indices[i]].r;
    point.g = cloud->points[inliers->indices[i]].g;
    point.b = cloud->points[inliers->indices[i]].b;
    inCloud->points.push_back(point);
  }
  return inCloud;
}


void color_filter (cloudPtr cursory_conversion, pcl::PointCloud<pcl::PointXYZRGB>* whiteCloud, pcl::PointCloud<pcl::PointXYZRGB>* redCloud, pcl::PointCloud<pcl::PointXYZRGB>* blueCloud)
{
	//remove any dark points first pcl::PointCloud
	pcl::PointCloud<pcl::PointXYZRGB> inCloud = *cursory_conversion;
	//pcl::PointCloud<pcl::PointXYZRGB> *outCloud = new pcl::PointCloud<pcl::PointXYZRGB>;
	for(int i = 0; i < inCloud.points.size(); i++) {
		if (inCloud.points[i].r>200 && inCloud.points[i].g>200 && inCloud.points[i].b>200 ){
			whiteCloud->points.push_back(inCloud.points[i]);
		}
		else if (inCloud.points[i].r<80 && inCloud.points[i].g<80 && inCloud.points[i].b>150 ){
			blueCloud->points.push_back(inCloud.points[i]);
		}
		else if (inCloud.points[i].r>150 && inCloud.points[i].g<80 && inCloud.points[i].b<80 ){
			redCloud->points.push_back(inCloud.points[i]);
		}
	}
		printf("Colors filtered\n");
}
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (outCloud);

	/*
	for ( std::vector<pcl::PointXYZRGB>::iterator pit = cloud->points.begin(); pit != cloud->points.end(); ++pit){
		//std::cout <<  cloud->points[i].r << " " << cloud->points[i].g << " " << cloud->points[i].b << std::endl;
		//printf("r:%c g:%c b:%c\n" , cloud->points[i].r, cloud->points[i].g, cloud->points[i].b );
		if ( cloud->points[*pit].r<200)//  cloud->points[i].g, cloud->points[i].b );
		{
			printf("erasing pointk");
			cloud->points.erase(pit);
		}
	}
	*/
void color_segment (cloudPtr cloud, pcl::PointCloud<pcl::PointXYZRGB> &whiteCloud, pcl::PointCloud<pcl::PointXYZRGB> &redCloud, pcl::PointCloud<pcl::PointXYZRGB> &blueCloud)
{
  float r_threshold = 1.0;
  float p_threshold = 1.0;
  int neighbors = 50;
  float d_thresh = 0.2;
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB, pcl::Normal> reg;
    std::cout << "Distance threshold: " << reg.getDistanceThreshold() << std::endl
      << "Point Threshold: " << reg.getPointColorThreshold() << "\n"
      << "Region Threshold: " << reg.getRegionColorThreshold() << "\n";

  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighbors);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setPointColorThreshold(p_threshold);
  reg.setRegionColorThreshold(r_threshold);
  reg.setCurvatureTestFlag(false);
  reg.setCurvatureThreshold(100.0);
  reg.setDistanceThreshold(100.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	int largest_cluster_size = 10;
	int largest_cluster_location = 0;
  for (int counter = 0; counter < clusters.size(); counter++)
  {
		if (clusters[counter].indices.size() > largest_cluster_size){
		 largest_cluster_size = clusters[counter].indices.size();
		 largest_cluster_location = counter;
		}
  }
	std::cout << "largest cluster= " << largest_cluster_size << " at location " << largest_cluster_location << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr white_line_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	//set iterator at largest cluster
	std::vector<pcl::PointIndices>::const_iterator it = clusters.begin()+largest_cluster_location;
	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		white_line_cluster->points.push_back (cloud->points[*pit]);
	}
	white_line_cluster->width = white_line_cluster->size ();
	white_line_cluster->height = 1;
	white_line_cluster->is_dense = true;

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  //viewer.showCloud(colored_cloud);
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud(white_line_cluster);

  return;
}

void test_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();


	 //--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
int count_points(float startx, float stopx, float starty, float stopy, float startz, float stopz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud) {
	int denied_counter = 0;
	int accepted_counter = 0;
	for(int i = 0; i < inCloud->points.size(); i++) {
		if (inCloud->points[i].x >= startx
				&& inCloud->points[i].x <= stopx
				&& inCloud->points[i].y >= starty
				&& inCloud->points[i].y <= stopy
				&& inCloud->points[i].z >= startz
				&& inCloud->points[i].z <= stopz)
		{
		  accepted_counter++;
		}
		else {
		  denied_counter++;
		}
	}
	//std::cout << "points in area: " << accepted_counter
	//<< " points denied: " << denied_counter
	//<< " ratio: " << (float)accepted_counter/(float)denied_counter*100 << "%" <<std::endl;
	return accepted_counter;
}

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr area_seg(float startx, float stopx, float starty, float stopy, float startz, float stopz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloud, std::string outname) {
void area_seg(float startx, float stopx, float starty, float stopy, float startz, float stopz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inCloud, pcl::PointCloud<pcl::PointXYZRGB> &outCloud, std::string outname) {

	std::cout << "area_seg begin" << std::endl
	<< "x, x2, y, y, z, z2:" << startx<<", " << stopx <<", " << starty <<", " << stopy <<", " << startz <<", " << stopz << std::endl;
	FILE             * pPointFile;

	int denied_counter = 0;
	int accepted_counter = 0;
	 printf("the outcloud is %d\n",outCloud.points.size());
	if (outCloud.size()>0)
		outCloud.clear();
	if ( inCloud->points.size() <= 1) { printf("This is an error. The incloud is empty\n"); return;}
	for(int i = 0; i < inCloud->points.size(); i++) {

		if (inCloud->points[i].x >= startx
				&& inCloud->points[i].x <= stopx
				&& inCloud->points[i].y >= starty
				&& inCloud->points[i].y <= stopy
				&& inCloud->points[i].z >= startz
				&& inCloud->points[i].z <= stopz)
		{
			//std::cout << "point accepted "<<std::endl;
			outCloud.points.push_back(inCloud->points[i]);
		  accepted_counter++;
		}
		else {
		  denied_counter++;
		}
	}
			std::cout << "points accepted: " << accepted_counter
			 << " points denied: " << denied_counter
			 << " ratio: " << (float)accepted_counter/(float)denied_counter*100 << "%" <<std::endl;
	//hacks to get around width*height error
		outCloud.width = 1;
		outCloud.height = accepted_counter;
			std::cout << "point cloud size: " << outCloud.size() << std::endl;
	// Save points to disk

			if (outname.compare("null")){//if filename is not null
				pPointFile = fopen( outname.c_str(), "w+" );
				pcl::io::savePCDFile (outname.c_str(), outCloud);

				int cursorLocation  = ftell(pPointFile);
				fseek(pPointFile, 0L, SEEK_END);
				int fileSize =ftell(pPointFile);
				fseek(pPointFile, 0L, cursorLocation);


				if ( pPointFile != NULL &&  fileSize>1) {
					//printf("Opening output file %s\n", outname);
					std::cout << "***saving*** " << outCloud.points.size() << " points" << std::endl;
					std::cout << "File " << outname << " written sucessfully"<<std::endl;
				}
				else {
					//printf("Error opening output file %s\n", outname);
					std::cout<<"Error opening output file"<< outname << std::endl;
				}
				fclose(pPointFile);
			}

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr Real_outCloud(&outCloud);//= new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(outCloud);
	//return Real_outCloud;
}

float * seg_info(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud) {
	std::cout<< "Seg Info" << std::endl;
	float highest_y=-100, highest_x=-100, highest_z=-100, lowest_x=100, lowest_y=100, lowest_z=100;
	//float info_array[6];
	float * info_array = new float [6];
	//for cloud
	for(int i =0; i<incloud->points.size(); i++){
		//get highest and lowest
		if ( incloud->points[i].x < lowest_x) {
			lowest_x= incloud->points[i].x;
			info_array[0]=lowest_x; }
		if ( incloud->points[i].x > highest_x) {
			highest_x= incloud->points[i].x;
			info_array[1]=highest_x; }
		if ( incloud->points[i].y < lowest_y) {
			lowest_y= incloud->points[i].y;
			info_array[2]=lowest_y; }
		if ( incloud->points[i].y > highest_y) {
			highest_y= incloud->points[i].y;
			info_array[3]=highest_y; }
		if ( incloud->points[i].z < lowest_z) {
			lowest_z= incloud->points[i].z;
			info_array[4]=lowest_z; }
		if ( incloud->points[i].z > highest_z) {
			highest_z= incloud->points[i].z;
			info_array[5]=highest_z;
		}
	}
	//print highest and lowest
	std::cout  << " lowest x:" << lowest_x << " highest x:" << highest_x << std::endl;
	std::cout  << " lowest y:" << lowest_y << " highest y:" << highest_y << std::endl;
	std::cout  << " lowest z:" << lowest_z << " highest z:" << highest_z << std::endl;
	std::cout  << "total points in area: " << incloud->points.size() << std::endl;
	std::cout  << "info end" << std::endl;

	return info_array;
}
//
//The rest of the code was contributed By sterling and handles camera access
//

// configue camera to capture image
int configureCamera( FC2::Camera & camera ) {
	FC2T::ErrorType fc2TriclopsError;
	FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );
    if ( fc2TriclopsError )
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    return 0;
}

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image& grabbedImage ) {
	FC2::Error fc2Error = camera.StartCapture();
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}

	fc2Error = camera.RetrieveBuffer(&grabbedImage);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}

	return 0;
}

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera     & camera,
                             TriclopsContext & triclops )
{
	FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}

	FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
		                                    "getContextFromCamera");
    }

	return 0;
}

int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage,
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput )
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage,
                            true /*assume little endian*/,
                            unprocessedImage[RIGHT],
                            unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                     "unpackUnprocessedRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image * monoImage = imageContainer.mono;

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE )
    {
        FC2::Image * bgruImage = imageContainer.bgru;

        for ( int i = 0; i < 2; ++i )
        {
            if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) )
            {
                return 1;
            }
        }

        FC2::PNGOption pngOpt;
        pngOpt.interlaced = false;
        pngOpt.compressionLevel = 9;
        bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
        bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);

        FC2::Image & packedColorImage = imageContainer.packed;

        // pack BGRU right and left image into an image
        fc2TriclopsError = FC2T::packTwoSideBySideRgbImage( bgruImage[RIGHT],
                                                            bgruImage[LEFT],
                                                            packedColorImage );

        if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
        {
            return handleFc2TriclopsError(fc2TriclopsError,
                                            "packTwoSideBySideRgbImage");
        }

        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput( grabbedImage.GetCols(),
                                                grabbedImage.GetRows(),
                                                packedColorImage.GetStride(),
                                                (unsigned long)grabbedImage.GetTimeStamp().seconds,
                                                (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
                                                packedColorImage.GetData(),
                                                &triclopsColorInput );

        _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );


        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions( packedColorImage.GetRows(),
                                        packedColorImage.GetCols(),
                                        packedColorImage.GetStride(),
                                        packedColorImage.GetPixelFormat(),
                                        FC2::NONE);

        packedColorImage.Save("packedColorImage.png",&pngOpt );

        for ( int i = 0; i < 2; ++i )
        {
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
            if (fc2Error != FlyCapture2::PGRERROR_OK)
            {
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }

        monoImage[RIGHT].Save("monoImageRight.pgm", &pgmOpt);
        monoImage[LEFT].Save("monoImageLeft.pgm", &pgmOpt);
    }
    else
    {
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }

    // Use the row interleaved images to build up an RGB TriclopsInput.
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput( grabbedImage.GetCols(),
                                        grabbedImage.GetRows(),
                                        grabbedImage.GetCols(),
                                        (unsigned long)grabbedImage.GetTimeStamp().seconds,
                                        (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
                                        monoImage[RIGHT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        &triclopsMonoInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    return 0;
}

// carry out stereo processing pipeline
int doStereo( TriclopsContext const & triclops,
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage )
{
    TriclopsError te;

    // Set subpixel interpolation on to use
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&stereoData) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

    // Retrieve the interpolated depth image from the context
    te = triclopsGetImage16( triclops,
                            TriImg16_DISPARITY,
                            TriCam_REFERENCE,
                            &depthImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Save the interpolated depth image
    char const * pDispFilename = "disparity16.pgm";
    te = triclopsSaveImage16( &depthImage, const_cast<char *>(pDispFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    return 0;
}

// save 3d points generated from stereo processing
pcl::PointCloud<pcl::PointXYZRGB>::Ptr save3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData,
                  std::string outname)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsError te;

    float            x, y, z;
    int	            r, g, b;
    FILE             * pPointFile;
    int              nPoints = 0;
    int	             pixelinc ;
    int	             i, j, k;
    unsigned short * row;
    unsigned short   disparity;

    // Rectify the color image if applicable
    bool isColor = false;
    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 )
    {
        isColor = true;
        te = triclopsRectifyColorImage( triclops,
                                        TriCam_REFERENCE,
	                                    const_cast<TriclopsInput *>(&colorData),
	                                    &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }
    else
    {
        te = triclopsGetImage( triclops,
	                            TriImg_RECTIFIED,
	                            TriCam_REFERENCE,
	                            &monoImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    }


    // Save points to disk
    pPointFile = fopen( outname.c_str(), "w+" );
    if ( pPointFile == NULL )
    {
				std::cout<< "Error opening output file ," << outname << std::endl;
    }

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    // Determine the number of pixels spacing per row
		pixelinc = disparityImage16.rowinc/2;
		for ( i = 0, k=0; i< disparityImage16.nrows; i++){
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
        {
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 )
                {
                    pcl::PointXYZRGB point;
                    if ( isColor )
                    {
                        r = (int)colorImage.red[k];
                        g = (int)colorImage.green[k];
                        b = (int)colorImage.blue[k];
                    }
                    else
                    {
                        // For mono cameras, we just assign the same value to RGB
                        r = (int)monoImage.data[k];
                        g = (int)monoImage.data[k];
                        b = (int)monoImage.data[k];
                    }
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.r = r;
                    point.g = g;
                    point.b = b;
                    cloud->push_back(point);
                    nPoints++;
                }
            }
        }
		}
    pcl::io::savePCDFile (outname.c_str(), *cloud);

    return cloud;
}
