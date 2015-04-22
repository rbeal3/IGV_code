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

	std::ifstream gpsData("gpsData.txt");
	std::ifstream gpsGoals("gpsGoals.txt");

	double * m_position = new double [6];
	double * goal_lat = new double [10];
	double * goal_long = new double [10];

	if (gpsData.is_open())
	{
		for(int info=0; info < 5; info++){
			getline(gpsData, str);
			//std::cout << "data from file:"  << str  << std::endl;
			str.erase(0, str.find(" "));
			m_position[info] = atof(str.c_str());
			//std::cout << "data in array:"  << m_position[info]  << std::endl;
		}
	}
	else{
		std::cout << "Error: No GPS file found!"  << std::endl;
	}
	//get number of goals
	int goals=0;
	while (std::getline(gpsGoals,str)) {
		goals++;
	}
	goals/=3;//3 lines per single entry

	std::cout << "There are " << goals << " GPS goals" << std::endl;
	gpsGoals.clear();
	gpsGoals.seekg(0, gpsGoals.beg);

	if (gpsGoals.is_open())
	{
		for(int goal=0; goal < goals; goal++){
			getline(gpsGoals, str);
			getline(gpsGoals, str);
			str.erase(0, str.find(" ")+1);
			goal_lat[goal]= atof(str.c_str());
			getline(gpsGoals, str);
			str.erase(0, str.find(" ")+1);
			goal_long[goal]= atof(str.c_str());
		}
	}
	else{
		std::cout << "Error: No goals file exists" << std::endl;
	}

	double my_bearing = 10;//m_position[3];
	std::cout << "don't forget to set bearing here!" << std::endl;

	for(int i=0; i<5; i++){
		std::cout << "position Info " << m_position[i] << std::endl;
	}
	for(int i=0; i<goals; i++){
		std::cout << "lat: " << goal_lat[i] << std::endl;
		std::cout << "lon: " << goal_long[i] << std::endl;
		
	}
	for(int i=0; i<goals; i++){

		double x_diff = m_position[1] - goal_lat[i]; //difference is positive if goal is west
		double y_diff = goal_long[i] - m_position[2]; //difference is positive if north

	//set bearing
		//double correct_angle = atan(x_difference/y_difference)*180/PI; 
		double correctBearing, Theta = atan2(y_diff, x_diff)*180/PI; 
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
		// tan = o/a

		std::cout << "new bearing " << correctBearing << std::endl;
	}
	return 0;
}
	/*
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
//
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
	*/

