#include "segmenters.h"
using namespace std;

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;
int TroubleCounter;

void inspect_robot(Robot* robo){
	for(int i=0; i<robo->goal_num; i++) {
		std::cout<< "goal_lat["<< i <<"] = " << robo->goal_lat[i] <<std::endl;
		std::cout<< "goal_long[" << i <<"] = " << robo->goal_long[i] <<std::endl;
	}

	std::cout<< "goalDistance" << robo->goalDistance << std::endl;
	std::cout<< "goalBearing" << robo->goalBearing << std::endl;
	std::cout<< "bearing" << robo->bearing <<std::endl;
	std::cout<< "latitude" << robo->latitude <<std::endl;
	std::cout<< "longitude" << robo->longitude <<std::endl;
	std::cout<< "checkDirection" << robo->checkDirection <<std::endl;
	std::cout<< "goal_num" << robo->goal_num <<std::endl;
}

int main( int argc, char**  argv)
{
	//init robot
	Robot* robot = new Robot();
	set_goals(robot);

	update_gps(robot);
	std::cout << "Initial gps: "  << std::endl;
	inspect_robot(robot);
	//initialize camera and context
	TriclopsContext triclops;
	FC2::Camera camera;
	int i = 0;
	if (argc==3) {
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1){
				PCL_ERROR ("Couldn't read the input file \n");
				return (-1);
			}
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
			std::cout << "floor cloud" << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB> *cloud_cut= new pcl::PointCloud<pcl::PointXYZRGB> ;
			pcl::PointCloud<pcl::PointXYZRGB> *whiteCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
			pcl::PointCloud<pcl::PointXYZRGB> *redCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
			pcl::PointCloud<pcl::PointXYZRGB> *blueCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;

			std::cout << "initial area_seg" << std::endl;
			area_seg(-5,5,.15,5,-5,5,cloud, *cloud_cut, "null");
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cut(cloud_cut);
			test_view(floor_cut);

			std::cout << "planar segmentation" << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segment;
			floor_segment = plainSeg(floor_cut);
			test_view(floor_segment);

			std::cout << "color segment: attempting to retrieve line segment" << std::endl;
				color_filter(floor_segment, whiteCloud, redCloud, blueCloud);
				cloudPtr whiteCloudPtr(whiteCloud);

			//test_view(whiteCloud);
			//compute_step(hazardCloud, robot);
	}
	else if (argc==2) {
			while(!at_goal(robot)){
			std::string command = "../bb2cloud/bb2cloud working_image.pcd";
			std::cout << "running sterlings camera " << std::endl;
			system(command.c_str());

			std::cout << std::endl;
			std::cout << "pulling cloud from file instead of camera" << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
			//if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("working_image.pcd", *cloud) == -1) {
				PCL_ERROR ("Couldn't read the input file \n");
				return (-1);
			}

			std::cout << "compute step" << std::endl;
			compute_step(cloud, robot);
		}
	}
	else{

		string outname = "out";//outname = argv[1];

		//setup
		TriclopsInput triclopsColorInput, triclopsMonoInput;
		//TriclopsContext triclops;//now decalared above for scope purposes
		//FC2::Camera camera;
		FC2::Image grabbedImage;

		camera.Connect();

		// configure camera
		if ( configureCamera( camera ) ) {
			return EXIT_FAILURE; }

		// generate the Triclops context
		if ( generateTriclopsContext( camera, triclops ) ) {
			return EXIT_FAILURE; }

		//MAIN PATHING LOOP
		int i =0;
		//while(++i<10)//!at_goal())// Not at GPS point?
		// grab image from camera.
		// this image contains both right and left images
		if ( grabImage( camera, grabbedImage ) ) {
			return EXIT_FAILURE; }

		ImageContainer imageContainer;

		// generate triclops inputs from grabbed image
		if ( generateTriclopsInput( grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput ) ) {
			return EXIT_FAILURE; }

		// output image disparity image with subpixel interpolation
		TriclopsImage16 disparityImage16;

		// carry out the stereo pipeline
		if ( doStereo( triclops, triclopsMonoInput, disparityImage16 ) ) {
			return EXIT_FAILURE; }

		// save text file containing 3d points
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = save3dPoints( grabbedImage, triclops, disparityImage16, triclopsColorInput, outname ) ;
		if (!cloud ) {
			return EXIT_FAILURE; }

		compute_step(cloud, robot);
	}

	// Close the camera and disconnect
	camera.StopCapture();
	camera.Disconnect();

	// Destroy the Triclops context
	TriclopsError te;
	te = triclopsDestroyContext( triclops ) ;
	_HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
	return 0;
}

int checkHazard( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int & left, int & center, int &right) {
	int CENTER =0,LEFT=1,RIGHT=2;
	float	botwidth=1;
	//note floor axis is inverted
	float floor_bot=.15;
	float floor_top= -5, range_max=.3, zmin=0.3, step_distance=1.1;

	int center_points = count_points(-botwidth/2, botwidth/2, floor_top, range_max, zmin, step_distance, cloud) ;
	int left_points = count_points ( -5, -botwidth/2, floor_top, range_max, zmin, step_distance, cloud) ;
	int right_points = count_points ( botwidth/2, 5, floor_top, range_max, zmin, step_distance, cloud) ;
	printf("\n");

	printf("Center: %d,  Left: %d , Right: %d \n", center_points, left_points, right_points);
//*
	pcl::PointCloud<pcl::PointXYZRGB>* cloudOut = new pcl::PointCloud<pcl::PointXYZRGB> ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cut(cloudOut);
	std::cout << "center area" << std::endl;
	area_seg ( -0.5, 0.5, floor_top, range_max, zmin, step_distance, cloud, *cloudOut, "null");
	//test_view(floor_cut);

	std::cout << "negative area" << std::endl;
	area_seg ( -5, -botwidth/2, floor_top, range_max, zmin, step_distance, cloud, *cloudOut, "null");
	//test_view(floor_cut);

	std::cout << "positive area" << std::endl;
	area_seg ( 0.5, 5, -5, 5, 0, step_distance, cloud, *cloudOut, "null");
	//test_view(floor_cut);
//*/

	if( center_points < 100 ) { //about empty
		center=0;
	}
	else{center=1;}

	if( left_points < 100 ) { //about empty
		left=0;
	}
	else{left=1;}
	if( right_points < 100 ) { //about empty
		right=0;
	}
	else{ right=1;}

	if(left_points < center_points && left_points < right_points){
	std::cout << "Left was smallest: " << std::endl;
			return LEFT;
	}
	else{
	std::cout << "Right was smallest: " << std::endl;
		return RIGHT;}
}

//returns 1 if within not noHazard( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
int at_goal(Robot* robot){
	//update gps values
	// 
	float GPS_diff=0;
	GPS_diff +=	abs(robot->longitude - robot->goal_long[ robot->goal_num]);
	GPS_diff +=	abs(robot->latitude - robot->goal_lat[robot->goal_num]);

	if (GPS_diff < .00005){//0.00001=1.1 meter
		std::cout <<"congrats you found a GPS point" <<std::endl;
		inspect_robot(robot);
		if(robot->goal_num>5)
		 return 1;
		else{
			robot->goal_num++;
			return 0;
		}
	}
	else {
		return 0;
	}
}
int gps_target(cloudPtr cloud, Robot* robot){
	std::string command ;
	int left=1, center=1, right=1;
	do{
		update_gps(robot);
		//Build the move command starting here
			command = "python ../utils/oneStep.py ";

			checkHazard(cloud, left, center, right);
			//turn towards GPS by default
				float diff = abs(int(robot->bearing-robot->goalBearing));
				if ((360>diff && diff >180) || (0 > diff && diff>-180)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-r ";
					command += arg2;
					 robot->bearing = int(robot->bearing-diff)%360;
				}
				else if((180> diff && diff>0) || (-180> diff && diff>-360)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-l ";
					command += arg2;
					if (robot->bearing-diff<0)
					 robot->bearing = 360-robot->bearing-diff;
					else
					 robot->bearing = robot->bearing-diff;
				}
				else if ((diff>-10 || diff<10) && center==0){
					command += "20 ";
				}
			std::cout << "gps wants (enter y to comply)"<<command << std::endl;
				char str;
				std::cin>> str;
			system(command.c_str());
	}while(center==0 && abs(robot->bearing-robot->goalBearing)>10);
	//if clear blocked, turn towards open vision
		std::cout << "center not clear"<<command << std::endl;
		command = "python ../utils/oneStep.py ";
	if (left==0){
		command += "-l 90";
		robot->checkDirection = RIGHT;
	}
	else if (right==0){
		command += "-r 90";
		robot->checkDirection = LEFT;
	}
			std::cout << "Gps asks for one final request before switching to vision(enter y to comply) "<<std::endl<<command << std::endl;
				char str;
				std::cin>> str;
		system(command.c_str());

	robot->method=VISION;
	return 0;
}

void vision_target(cloudPtr cloud, Robot* robot){
	
		int left=1, center=1, right=1;
    checkHazard(cloud, left, center, right);

		std::string command = "python ../utils/oneStep.py ";

		if (robot->checkDirection==LEFT && left==0) {
		 command = "python ../utils/oneStep.py -l 60";
			//overshoot to check really clear
			//take new pic
			//checkHazard();//fuck, we need a new image for this
			//if (center==0 && left==0){
			 //command = "python ../utils/oneStep.py -r 30 ";
			 //command = "python ../utils/oneStep.py 20 ";

		}
		else if (robot->checkDirection==RIGHT && right==0) {
			command += "-r 30";
		}
    if(center==0) { //center clear
		 command = "python ../utils/oneStep.py 20 ";
    }
		if(left+right+center==3){//no way out 
			if (robot->checkDirection==RIGHT){
				command = "python ../utils/oneStep.py -l 140";
				robot->checkDirection=LEFT;
			}
			
			if (robot->checkDirection==LEFT){
				command = "python ../utils/oneStep.py -r 140";
				robot->checkDirection=RIGHT;
			}
		}

		std::cout << "going to move "<<command << std::endl<< "Enter y if this is ok"<<std::endl;
		char ok;
		std::cin >> ok;
		if (ok=='y'){
			system(command.c_str());
		}
		else{
			stringstream ss;
			ss << TroubleCounter;
			string tc = ss.str();
			//copy cloud to troubleshooting folder
			command="cp working_image"+tc+".pcd ";
			system(command.c_str());
		}

}
//compute path from cloud and take a step
void compute_step(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Robot* robot){

	if(robot->method==GPS)
		gps_target(cloud, robot);
	else if (robot->method==VISION)
		vision_target(cloud, robot);

}

int convertPCDtoMAP(int pcd)
{
	//decide range 1200*2400in = 28,80,000sqin/16sqin = 180,000 total cells

	//generate pass map
	//for range
	////for each set of pixels
	//else if (y_difference < 0 && x_difference > 0 ){ //goal is southwest
		//correct_bearing=correct_angle+180;
	//}

	//print how many pixels
	//if pixles < normal*.5
		//set cell as unsafe


	//(avg points in range)
	return 0;
}
