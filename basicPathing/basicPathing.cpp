#include "segmenters.h"
using namespace std;

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;
int TroubleCounter;
bool simple, debug, debug_short, testing;
void vision_target(cloudPtr cloud, Robot* robot);

void inspect_robot(Robot* robo){
	for(int i=0; i<robo->goal_num; i++) {
		std::cout<< "goal_lat["<< i <<"] = " << robo->goal_lat[i] <<std::endl;
		std::cout<< "goal_long[" << i <<"] = " << robo->goal_long[i] <<std::endl;
	}

	std::cout<< "goalDistance " << robo->goalDistance << std::endl;
	std::cout<< "goalBearing " << robo->goalBearing << std::endl;
	std::cout<< "bearing " << robo->bearing <<std::endl;
	std::cout<< "latitude " << robo->latitude <<std::endl;
	std::cout<< "longitude " << robo->longitude <<std::endl;
	std::cout<< "checkDirection " << robo->checkDirection <<std::endl;
	std::cout<< "goal_num " << robo->goal_num <<std::endl;
}

int main( int argc, char**  argv)
{
	//init robot
	Robot* robot = new Robot();
	set_goals(robot);

	std::cout << "Initial gps: "  << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segment;
		pcl::PointCloud<pcl::PointXYZRGB> inCloud= *cloud;
		pcl::PointCloud<pcl::PointXYZRGB> *cloud_cut= new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB> *whiteCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB> *redCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB> *blueCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB> *hazardCloud= new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB> *above_floor_segment= new pcl::PointCloud<pcl::PointXYZRGB> ;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > extracted_line_hazards;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>* > extracted_hazards;
	if (argc==3) {
			if( strcmp(argv[2], "-d") == 0 ) debug = true;
			if( strcmp(argv[2], "-s") == 0 ) debug_short = true;
			if( strcmp(argv[2], "-t") == 0 ) testing = true;
			if( strcmp(argv[2], "-min") == 0 ) simple= true;

			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1){
				PCL_ERROR ("Couldn't read the input file \n");
				return (-1);
			}
			std::cout << "floor cloud" << std::endl;

			//here we may want to ignore data sets that have objects very close to the camera
			//as they may contain significant noise.

			cloud_cut = area_seg(-5,5,.15,5,-5,5,cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(cloud_cut);
			if (debug){
				std::cout << "initial area_seg" << std::endl;
				test_view(cloud_ptr);
			}

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segment;
			floor_segment = plainSeg(cloud_ptr);

			if (debug){
				std::cout << "planar segmentation" << std::endl;
				test_view(floor_segment);
			}

			whiteCloud= color_filter(floor_segment, redCloud, blueCloud);
			cloudPtr whiteCloudPtr(whiteCloud);
			if (debug||debug_short){
				std::cout << "color segment: attempting to retrieve line segment" << std::endl;
				test_view(whiteCloudPtr);
			}
			//filter out noise
			extracted_line_hazards = noiseFilter(whiteCloudPtr);
			if (debug){
				for (int i= 0; i < extracted_hazards.size(); i++){
					std::cout << "lines["<< i <<"]!" << std::endl;
					cloud_ptr= extracted_line_hazards[i]->makeShared();
					test_view(cloud_ptr);
				}
			}

			//now for the cans
			cloud_cut = area_seg(-5,5,-5,.2,-5,5,cloud);
			cloud_ptr= cloud_cut->makeShared();
			if (debug){
				std::cout << "initial area_seg" << std::endl;
				//cloud_ptr already points at cloud_cut
				test_view(cloud_ptr);
			}
			extracted_hazards = regionSeg(cloud_ptr);

			if (debug){
				for (int i=0; i < extracted_hazards.size(); i++){
					std::cout << "hazards["<< i <<"]!" << std::endl;
					cloud_ptr=extracted_hazards[i]->makeShared();
					test_view(cloud_ptr);
				}
			}
			//compile hazards
			float object_info;
			for (int i=0; i < extracted_hazards.size(); i++){
				*hazardCloud += *extracted_hazards[i];
				object_info = saveAnd_condense_object(robot, extracted_hazards[i]);
			}
			*hazardCloud += *whiteCloud;
			cloud_ptr = hazardCloud->makeShared();

			if (debug || debug_short){
				std::cout << "hazard Cloud" << std::endl;
				test_view(cloud_ptr);
			}
			//compute_step(hazardCloud, robot);
			vision_target(cloud_ptr, robot);//as alternate to compute (uses GPS)
	}
	else if (argc==2) {
		if( strcmp(argv[1], "-t") == 0 ) testing = true;
		if( strcmp(argv[1], "-d") == 0 ){
			debug = true;
			robot->method=VISION;
		}
		if( strcmp(argv[1], "-s") == 0 ) simple = true;
		if( strcmp(argv[1], "vis") == 0 ) robot->method=VISION;
		if(robot->method!=VISION)
			update_gps(robot);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
		while(!at_goal(robot)){
			std::string command = "../bb2cloud/bb2cloud working_image.pcd";
			std::cout << "running sterlings camera " << std::endl;
			int ret_value;
			ret_value = system(command.c_str());
			std::cout << "sterlings code returned: "<< ret_value << std::endl;

			std::cout << std::endl;
			std::cout << "pulling cloud from file instead of camera" << std::endl;
			//if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("working_image.pcd", *cloud) == -1) {
				PCL_ERROR ("Couldn't read the input file \n");
				return (-1);
			}
			if(cloud->points.size()<=1){
				std::cout<< "cloud too small.Camera likely failed"<< std::endl;
			}

			//here we may want to ignore data sets that have objects very close to the camera
			//as they may contain significant noise.


			cloud_cut = area_seg(-5,5,.15,5,-5,5,cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(cloud_cut);
			if (debug){
				std::cout << "initial area_seg viewer loading" << std::endl;
				test_view(cloud_ptr);
			}
//SEG FAULT seg fault just below...
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segment;//moved cause segFault 

			floor_segment = plainSeg(cloud_ptr);
			std::cout<<"cloud_ptr Size " << cloud_ptr->size()<<std::endl;
			std::cout<<"floorSeg Size " << floor_segment->size()<<std::endl;

			if (debug){
				std::cout << "planar segmentation" << std::endl;
				test_view(floor_segment);
			}
		/* if (whiteCloud->size() > 0){ 
				for ( pcl::PointCloud<pcl::PointXYZRGB>::iterator pit = whiteCloud->points.begin(); pit != whiteCloud->points.end(); ++pit){
						whiteCloud->points.erase(pit); } //whiteCloud->clear(); } */
			whiteCloud = color_filter(floor_segment, redCloud, blueCloud);
			cloudPtr whiteCloudPtr(whiteCloud);
			if (debug||debug_short){
				std::cout << "color segment: attempting to retrieve line segment" << std::endl;
				test_view(whiteCloudPtr);
			}
			if (whiteCloud->points.size()!=0) { 
				//filter out noise
				extracted_line_hazards = noiseFilter(whiteCloudPtr);
				if (debug){
					for (int i=0; i < extracted_hazards.size(); i++){
						std::cout << "lines["<< i <<"]!" << std::endl;
						cloud_ptr=extracted_line_hazards[i]->makeShared();
						test_view(cloud_ptr);
					}
				}
			}
			//now for the cans
			cloud_cut = area_seg(-5,5,-5,.2,-5,5,cloud);
			cloud_ptr=cloud_cut->makeShared();
			if (debug){
				std::cout << "initial area_seg" << std::endl;
				//cloud_ptr already points at cloud_cut
				test_view(cloud_ptr);
			}
			extracted_hazards = regionSeg(cloud_ptr);

			if (debug){
				for (int i=0; i < extracted_hazards.size(); i++){
					std::cout << "hazards["<< i <<"]!" << std::endl;
					cloud_ptr=extracted_hazards[i]->makeShared();
					test_view(cloud_ptr);
				}
			}
			//compile hazards
			hazardCloud->clear();
			for (int i=0; i < extracted_hazards.size(); i++){
				*hazardCloud += *extracted_hazards[i];
			}
//SEG FAULT seg fault just below...
			//if there is no hazard... watch out...
			std::cout<<"hazardCloud Size" << hazardCloud->size()<<std::endl;
			std::cout<<"whiteCloud Size" << whiteCloud->size()<<std::endl;
			*hazardCloud+= *whiteCloud;
			cloud_ptr = hazardCloud->makeShared();

			if (debug || debug_short){
				std::cout << "hazard Cloud" << std::endl;
				test_view(cloud_ptr);
			}
			//compile hazards
			std::cout << "compute step" << std::endl;
			cloud_ptr = hazardCloud->makeShared();
			compute_step(cloud_ptr, robot);
		}
	}
	else{

    TriclopsInput triclopsColorInput, triclopsMonoInput;
    TriclopsContext triclops;
    FC2::Camera camera;
    FC2::Image grabbedImage;
    camera.Connect();

    // configure camera
    if ( configureCamera( camera ) ) {
			std::cout << "ERROR with configure camera. exiting"<<std::endl;
			return EXIT_FAILURE;
    }

    // generate the Triclops context 
    if ( generateTriclopsContext( camera, triclops ) ) {
			std::cout << "ERROR with generate context. exiting"<<std::endl;
			return EXIT_FAILURE;
    }
			// Container of Images used for processing
			ImageContainer imageContainer;

    // grab image from camera.
    // this image contains both right and left images
		while(!at_goal(robot)){
			if ( grabImage( camera, grabbedImage ) ) {
				std::cout << "ERROR with grab Image. exiting"<<std::endl;
				return EXIT_FAILURE;
			}


			// generate triclops inputs from grabbed image
			if ( generateTriclopsInput( grabbedImage, 
																	imageContainer,
																	triclopsColorInput, 
																	triclopsMonoInput ) 
				 )
			{
			std::cout << "ERROR with camera:gTI exiting"<<std::endl;
			return EXIT_FAILURE;
			}

			// output image disparity image with subpixel interpolation
			TriclopsImage16 disparityImage16;

			// carry out the stereo pipeline 
			if ( doStereo( triclops, triclopsMonoInput, disparityImage16 ) ) { std::cout << "ERROR with stereo pipeline. exiting"<<std::endl;
			return EXIT_FAILURE;
			}

			// save text file containing 3d points
				save3dPoints( grabbedImage, triclops, disparityImage16, triclopsColorInput);
			if (0) {
				std::cout << "ERROR with compiling image. exiting"<<std::endl;
				return EXIT_FAILURE;
			}
		}

	// Close the camera and disconnect
	camera.StopCapture();
	camera.Disconnect();
 
	// Destroy the Triclops context
	TriclopsError te;
	te = triclopsDestroyContext( triclops ) ;
	_HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );

	}
	return 0;
}

//Distance is how far out hazards should be counted in meters
int checkHazard( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float distance, int & left, int & center, int &right) {
	int CENTER =0,LEFT=1,RIGHT=2;
	float	botwidth=1;
	//note floor axis is inverted
	float floor_bot=3;//.15;
	float floor_top= -5, zmin=0;

	int center_points = count_points(-botwidth/2, botwidth/2, floor_top, floor_bot, zmin, distance, cloud) ;
	int left_points = count_points ( -5, -.25 , floor_top, floor_bot, zmin, distance, cloud) ;
	int right_points = count_points ( .25 , 5, floor_top, floor_bot, zmin, distance, cloud) ;
	printf("\n");

	printf("Center: %d,  Left: %d , Right: %d \n", center_points, left_points, right_points);
		/*
	if(debug){
		pcl::PointCloud<pcl::PointXYZRGB>* cloudOut = new pcl::PointCloud<pcl::PointXYZRGB> ;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cut(cloudOut);
		std::cout << "center area" << std::endl;
		area_seg ( -0.5, 0.5, floor_top, floor_bot, zmin, step_distance, cloud, *cloudOut, "null");
		test_view(floor_cut);

		std::cout << "negative area" << std::endl;
		area_seg ( -5, -botwidth/2, floor_top, floor_bot, zmin, step_distance, cloud, *cloudOut, "null");
		test_view(floor_cut);

		std::cout << "positive area" << std::endl;
		area_seg ( 0.5, 5, -5, 5, 0, step_distance, cloud, *cloudOut, "null");
		test_view(floor_cut);
	}
		*/

	if( center_points < 150*distance ) { //about empty
		std::cout << "center_points" << center_points << std::endl;
		center=0;
	}
	else{center=1;}

	if( left_points < 100 ) { //about empty
		std::cout << "left_points" << center_points << std::endl;
		left=0;
	}
	else{left=1;}
	if( right_points < 100 ) { //about empty
		std::cout << "right_points" << center_points << std::endl;
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
	float GPS_diff=0;
	GPS_diff +=	abs(robot->longitude - robot->goal_long[ robot->goal_num]);
	GPS_diff +=	abs(robot->latitude - robot->goal_lat[robot->goal_num]);

	if (GPS_diff < .00002 && (robot->latitude!=0 || robot->longitude!=0)){//0.00001=1.1 meter
		std::cout <<"congrats you found a GPS point" <<std::endl;
		inspect_robot(robot);
		if(robot->goal_lat[robot->goal_num]= 0){//only return true when there are no more points
			return 1;
		}
		else{ //else iterate through to the next goal
			robot->goal_num++;
			return 0;
		}
	}
	else {
		return 0;
	}
}
int gps_simple(cloudPtr cloud, Robot* robot){
	std::string command ;
	int left=1, center=1, right=1, smallest=CENTER;
	do{
		update_gps(robot);
		if(robot->longitude=0) return 1;
		std::cout << "My Bearing: "<<robot->bearing << std::endl
			<< "Goal Bearing: "<< robot->goalBearing  << std::endl;

		//Build the move command starting here
			command = "python ../utils/oneStep.py ";

			checkHazard(cloud, 1.4, left, center, right);
			//turn towards GPS by default
				float diff = abs(int(robot->bearing-robot->goalBearing));
				if ((360>diff && diff >180) || (0 > diff && diff>-180)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-r ";
					command += arg2;
					 robot->bearing = int(robot->bearing+diff)%360;
				}
				else if((180> diff && diff>0) || (-180> diff && diff>-360)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-l ";
					command += arg2;
					robot->bearing = robot->bearing -diff;
					if (robot->bearing< 0)
					 robot->bearing = 360+robot->bearing;
				}
				else if ((diff>-10 || diff<10) && center==0){
					command += "40 ";
				}
			std::cout << "diff: "<< diff << std::endl;
			std::cout << "gps wants "<<command << std::endl;
			if (testing){
				std::cout << "(enter y to comply)"<<command << std::endl;
				char str;
				std::cin>> str;
			}
			system(command.c_str());
	}while(center==0 || abs(robot->bearing - robot->goalBearing) >10);

	robot->method=VISION;
	return 0;
}
int gps_target(cloudPtr cloud, Robot* robot){
	std::string command ;
	int left=1, center=1, right=1, smallest=CENTER;
	do{
		update_gps(robot);
		if(robot->longitude=0) return 1;

		//Build the move command starting here
			command = "python ../utils/oneStep.py ";

			checkHazard(cloud, 1.4, left, center, right);
			//turn towards GPS by default
				float diff = abs(int(robot->bearing-robot->goalBearing));
				if ((360>diff && diff >180) || (0 > diff && diff>-180)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-r ";
					command += arg2;
					 robot->bearing = int(robot->bearing+diff)%360;
				}
				else if((180> diff && diff>0) || (-180> diff && diff>-360)){
					stringstream ss;
					ss << diff;
					string arg2 = ss.str();
					command += "-l ";
					command += arg2;
					robot->bearing = robot->bearing -diff;
					if (robot->bearing< 0)
					 robot->bearing = 360+robot->bearing;
				}
				else if ((diff>-10 || diff<10) && center==0){
					command += "40 ";
				}
			std::cout << "diff: "<< diff << std::endl;
			std::cout << "gps wants "<<command << std::endl;
			if (testing){
				std::cout << "(enter y to comply)"<<command << std::endl;
				char str;
				std::cin>> str;
				if (str=='y'){
				system(command.c_str());
				}
			}
			else{
				system(command.c_str());
			}
	}while(center==0 || abs(robot->bearing - robot->goalBearing) >10);

	//if clear blocked, turn towards open vision
		std::cout << "center not clear, but bearing correct"<<std::endl
			<<"switch to vision after turning:"<<std::endl;
	if (smallest==LEFT){
		command = "python ../utils/oneStep.py -l 90";
		robot->checkDirection = RIGHT;
	}
	else {
		command = "python ../utils/oneStep.py -r 90";
		robot->checkDirection = LEFT;
	}
	
			std::cout << "Gps asks for one final request before switching to vision "<<std::endl<<command << std::endl;
	if (testing){
			std::cout << " enter y to comply "<<std::endl;
				char str;
				std::cin>> str;
				if (str=='y'){
				system(command.c_str());
				}
	}
		else{
			system(command.c_str());
		}

	robot->method=VISION;
	return 0;
}

void vision_simple(cloudPtr cloud, Robot* robot){
	
	//assume we are just here to avoid an object
	//assume further that we have already turned,

	int center_near = count_points(-.5, .5, -5, 3, 0, 1, cloud);
	int center_med = count_points(-.5, .5, -5, 3, 0, 2, cloud);
	int center_far = count_points(-.5, .5, -5, 3, 0, 3, cloud);

	std::cout << "center_near " << center_near
	<< "center_med " << center_med
	<< "center_far " << center_far << std::endl;
		if(debug)
			test_view(cloud);

		std::string command = "";
		//checking assumes we have turned and gotten a new pic 
		 if(center_far<200) { //center clear
		 command = "python ../utils/oneStep.py 90 ";
    }
		else if(center_med<200) { //center clear
		 command = "python ../utils/oneStep.py 60 ";
    }
		else if(center_near<200) { //center clear
		 command = "python ../utils/oneStep.py 30 ";
    }
		else {
		 command = "python ../utils/oneStep.py -l 30 ";
		 //I would like to do a few more iterations before switching to gps
			robot->method=GPS;
		}

		std::cout << "going to move "<<command << std::endl;
	if (testing){
		std::cout << "Enter y if this is ok"<<std::endl;
		char ok;
		std::cin >> ok;
		if (ok=='y'){
			system(command.c_str());
		}
		if (ok=='c'){
		 command = "python ../utils/oneStep.py 30 ";
			system(command.c_str());
		 command = "python ../utils/oneStep.py -r 30 ";
			system(command.c_str());
		}
		else{
			stringstream ss;
			ss << TroubleCounter;
			string tc = ss.str();
			//copy cloud to troubleshooting folder
			command="cp working_image.pcd log/working_image"+tc+".pcd ";
		}
	}
	else{
			system(command.c_str());
	}

}
void vision_target(cloudPtr cloud, Robot* robot){
	
	//assume we are just here to avoid an object
	//assume further that we have already turned,
		int left=1, center=1, right=1;
		
    checkHazard(cloud, 1.4, left, center, right);
		if(debug)
			test_view(cloud);

		std::string command = "";
		//checking assumes we have turned and gotten a new pic 
		if (robot->checking && robot->checkDirection==LEFT) {
			robot->check_result=center+right;
		}
		else if (robot->checking && robot->checkDirection==RIGHT) {
			robot->check_result=center+left;
		}
		if (robot->checking && robot->check_result==0) {//if robot is checking side, and path was clear
			//turn back and go forward
			if (robot->checkDirection==RIGHT) {
				command = "python ../utils/oneStep.py -l 30";
				system(command.c_str());
				std::cout<<"make sure system halts here momentairly after left turn, and then goes forward 3 ft" << std::endl;
				std::cout<<"(testing our max command send speed)" << std::endl;
				//modify bearing when turning
				robot->bearing = robot->bearing -30;
				if (robot->bearing< 0)
				 robot->bearing = 360+robot->bearing;
				}
				command = "python ../utils/oneStep.py 30";
				system(command.c_str());
				robot->checking=0;

			if (robot->checkDirection==LEFT) {
				command = "python ../utils/oneStep.py -r 30";
				system(command.c_str());
				std::cout<<"make sure system halts here momentairly after left turn, and then goes forward 3 ft" << std::endl;
				std::cout<<"(testing our max command send speed)" << std::endl;
				//modify bearing when turning
				robot->bearing = ((int)robot->bearing +30)%360;

				command = "python ../utils/oneStep.py 30";
				system(command.c_str());
				robot->checking=0;
			}
		}
		else if (robot->checking==1)//checking, but path was not clear turn back
		{
			if (robot->checkDirection==RIGHT) {
				command = "python ../utils/oneStep.py -l 60";
				system(command.c_str());
				robot->checking=0;

				//modify bearing when turning
				robot->bearing = robot->bearing - 60;
				if (robot->bearing< 0)
				 robot->bearing = 360+robot->bearing;
			}
			else if (robot->checkDirection==LEFT) {
				command = "python ../utils/oneStep.py -r 60";
				system(command.c_str());
				robot->bearing = ((int)robot->bearing +30)%360;
				robot->checking=0;
			}
		}

		//check turns first since the whole purpose of vis is to avoid stuff
		if (robot->checkDirection==LEFT && left==0) {
			command = "python ../utils/oneStep.py -l 60";
			robot->checking=1;
			robot->check_result=-1;//unchecked

			//modify bearing when turning
			robot->bearing = robot->bearing -60;
			if (robot->bearing< 0)
			 robot->bearing = 360+robot->bearing;
		}
		else if (robot->checkDirection==RIGHT && right==0) {
		 command = "python ../utils/oneStep.py -r 60";
			robot->checking=1;
			robot->check_result=-1;//unchecked
			//modify bearing when turning
			robot->bearing = ((int)robot->bearing +60)%360;
		}
		else if(center==0) { //center clear
		 command = "python ../utils/oneStep.py 20 ";
    }
		else if(left+right+center==3){//no way out 
			std::cout << "No way out!"<< std::endl;
			if (robot->checkDirection==RIGHT){
				command = "python ../utils/oneStep.py -l 140";
				robot->checkDirection=LEFT;
				//modify bearing when turning
				robot->bearing = robot->bearing -140;
				if (robot->bearing< 0)
				 robot->bearing = 360+robot->bearing;
			}
			
			if (robot->checkDirection==LEFT){
				command = "python ../utils/oneStep.py -r 140";
				robot->checkDirection=RIGHT;
				//modify bearing when turning
				robot->bearing = ((int)robot->bearing +140)%360;
			}
		}
		if (command.size()<3){//command never set
			std::cout << "weird edge case!!!! command never set. BUG"<< std::endl;
			checkHazard(cloud, 0.7, left, center, right);
			if(center==0)
			 command = "python ../utils/oneStep.py 20 ";
			if(left==0)
			 command = "python ../utils/oneStep.py -l 35 ";
			if(right==0)
			 command = "python ../utils/oneStep.py -r 30 ";
		}

		std::cout << "going to move "<<command << std::endl;
	if (testing){
		std::cout << "Enter y if this is ok"<<std::endl;
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
			command="cp working_image.pcd log/working_image"+tc+".pcd ";
			system(command.c_str());
		}
	}
	else{
			system(command.c_str());
	}

}
//compute path from cloud and take a step
void compute_step(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Robot* robot){

	if(robot->method==GPS)
		gps_simple(cloud, robot);
		//gps_target(cloud, robot);
	else if (robot->method==VISION)
		vision_simple(cloud, robot);
		//vision_target(cloud, robot);

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
