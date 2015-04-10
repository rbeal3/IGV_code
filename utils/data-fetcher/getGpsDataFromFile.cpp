#include <iostream>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <string>
#define PI 3.141592653979851

int goto_point();
int main(){
	std::string  str;
	//latitude, longitude, bearing, altitude, speed, time,

	std::ifstream myfile("gpsData.txt");

	double * m_position = new double [6];
	if (myfile.is_open())
	{
		for(int info=0; info <=5; info++){
			getline(myfile, str);
			std::cout << " "  << str  << std::endl;
			str.erase(0, str.find("=")+1);
			m_position[info] = atof(str.c_str());
		}
	}
		std::string str_desired_longitude, str_desired_latitude;
		double desired_longitude, desired_latitude;
		//get gps from user
		std::cout << "enter desired latitude" << std::endl;
		getline(std::cin, str_desired_latitude);
		std::cout << "enter desired longitude" << std::endl;
		getline(std::cin, str_desired_longitude);

		 desired_latitude = atof(str_desired_latitude.c_str());
		 desired_longitude = atof(str_desired_longitude.c_str());

	double my_bearing = 10;//m_position[3];
	std::cout << "don't forget to set bearing here!" << std::endl;

	double x_difference = m_position[0] - desired_latitude; //difference is positive if goal is west
	double y_difference = desired_longitude - m_position[1]; //difference is positive if north

//set bearing
	double correct_bearing, correct_angle = atan(x_difference/y_difference)*180/PI; 
	// tan c_a = o/a

	std::cout << "calculation check:!" << std::endl;
	std::cout << "x_difference: "<< x_difference << std::endl;
	std::cout << "y_difference: "<< y_difference << std::endl;
	std::cout << "correct_angle: "<< correct_angle << std::endl;
	if (y_difference >= 0 && x_difference >= 0 ) { //goal is northeast
		correct_bearing=correct_angle-360;
	}
	else if (y_difference >= 0 && x_difference <= 0 ){ //goal is northwest
		correct_bearing=correct_angle;
	}
	else if (y_difference <= 0 && x_difference >= 0 ){ //goal is southwest
		correct_bearing=correct_angle+180;		
	}
	else if (y_difference <= 0 && x_difference >= 0 ){ //goal is southeast
		correct_bearing=correct_angle-180;		
	}

	std::cout << "new bearing " << correct_bearing << std::endl;

//handels driving after bearing is determined
	if(my_bearing == correct_bearing){
		//stay true, go straight
		}
	else if (((int)my_bearing+180)%360 > correct_bearing )
	{
		//turn right
	}
	else 
	{ 
		//turn left
	}

	return 0;
}
