
//Author Ryan Beal

#include <iostream>
#include <sstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

/*
*
*
*	objective: go from a to b without hitting anything
*	create an object that represents an area in memory. 
*	for now assume we start in the middle bottom facing north.
*
*	//method mapView
*	current gps location
*	approximate reletive location of every point
*
*
* 	Aquire floor
*		Perform color seg
*		add cloud to hazard map
*
*
*	make a handgps
*
*/
struct hazard{
	float location;
};
struct line_hazard{
};
void point_to_gps(float myGps, float bearing){
	
	//point north 
	//conversion 
	//if bearing is north
		//then z is strictly additive
	//if bearing is east 90
	//convert reletive bearing to 
	 //then z to the right (pos) is actully minus the longetude.

};
int main(int argc, char** argv)
{
	
	return 0;
}
