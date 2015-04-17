#include "segmenters.h"
using namespace std;

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct node {
	//	vector of pointers neighbors;
	//	location(s)
};
struct Robot_singleton{

	private:
		static Robot_singleton * instance;
		Robot_singleton(){}
		Robot_singleton(const Robot_singleton & source){}//disabling copy constructor
		//Robot_singleton(const Robot_singleton && source){}//disabling move constructor
		//Robot_singleton(Robot_singleton const &)= delete;
		//Robot_singleton(Robot_singleton&&)= delete;
		std::vector< std::vector<float> > goal;

		float position;
	public:
		void addGoal(float lat, float lon){
			this->goal[0].push_back(lat);
			this->goal[1].push_back(lon);
		}
		Robot_singleton * getInstance() {
			if (instance ==NULL){
				instance = new Robot_singleton();
			}
			return instance;
		}
};

int main( int argc, char**  argv)
{
	//initialize camera and context
	TriclopsContext triclops;
	FC2::Camera camera;

	if (argc==3) {
		std::cout << "one arg. pulling cloud from file instead of camera" << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1)
		{
			PCL_ERROR ("Couldn't read the input file \n");
			return (-1);
		}

		std::cout << "floor cloud" << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB> *cloud_cut= new pcl::PointCloud<pcl::PointXYZRGB> ;
		std::cout << "initial area_seg" << std::endl;
		area_seg(-5,5,.15,5,-5,5,cloud, *cloud_cut, "null");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cut(cloud_cut);
		test_view(floor_cut);

		std::cout << "planar segmentation" << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_segment;
		floor_segment = plainSeg(floor_cut);
		test_view(floor_segment);

		std::cout << "color segment" << std::endl;
		test_view(color_segment(floor_segment));

		std::cout << "compute step" << std::endl;
		compute_step(cloud);
		return 0;
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

		compute_step(cloud);
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

int noHazard( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	int CENTER =0,LEFT=1,RIGHT=2;
	float	botwidth=1;
	//note floor axis is inverted
	float floor_bot=.15;
	float floor_top= -5, range_max=.3, zmin=0.3, step_distance=2;


	printf("Center area:");
	int center_points = count_points(-botwidth/2, botwidth/2, floor_top, range_max, zmin, step_distance, cloud) ;
	printf("Left area: ");
	int left_points = count_points ( -5, -botwidth/2, floor_top, range_max, zmin, step_distance, cloud) ;
	printf("Right area: ");
	int right_points = count_points ( botwidth/2, 5, floor_top, range_max, zmin, step_distance, cloud) ;
	printf("\n");

	printf("Center: %d,  Left: %d , Right: %d \n", center_points, left_points, right_points);


	if( center_points < 100 ) { //about empty
		return CENTER;}
	else if( left_points < 100 ) { //about empty
		 return LEFT; }
	else if( right_points < 100 ) { //about empty
		return RIGHT;}
	//std::cout << "No area was small enough. The smallest area was: " << std::endl;
	else if (center_points < right_points && center_points < left_points){
			return CENTER;}
	else if(left_points < center_points && left_points < right_points){
			return LEFT; }
	else{return RIGHT;}
}

//returns 1 if within not noHazard( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
int at_goal(){
	int GPS_difference = 100; // Robot_settings.lat - ;
	if (GPS_difference < 5){
	 return 1;
	}
	else {
		return 0;
	}
}
//compute path from cloud and take a step
void compute_step(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	//ways to determine if there is space in front of me

		//create cloud that noHazard will run on
		//create thread
		//first thread handles white line

		//second thread handles objects

    //if there is no hazard in front of me, go straight
    int path_clear = noHazard(cloud);
		std::cout << "path clear:" << path_clear << std::endl;
		std::string command = "python ";

		std::string filename = "../utils/oneStep.py";
		command += filename;

    if(path_clear==CENTER) {
				command += " 0";
				std::cout<< "blazing straight ahead" << std::endl;
    }
    else { //turn somewhere better
        int turnAngle=60;//camera angle width
				int turnFactor= 10000;//about 35 degres
				turnAngle*=turnFactor;

				stringstream ss;
				ss << turnAngle;
				string arg2 = ss.str();

				if (path_clear==LEFT) {
					std::cout<< "turning left" << std::endl;
					command += "-l";
					command += arg2;
				}
				else if (path_clear==RIGHT) {
					std::cout<< "turning right" << std::endl;
					command += "-r";
					command += arg2;
				}
				else if (path_clear==NONE)
				{//turn towards gps
				//if i am already pretty close to gps direction default turn around
				//never back up or turn unless you know you are not close enough to hit something.
				}

    }
		//system(command.c_str());
		std::cout<< "not connected, command not sent " << std::endl;
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
