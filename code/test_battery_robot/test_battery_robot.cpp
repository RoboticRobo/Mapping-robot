
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = false;

int main()
{
	CreateData	robotData;
	RobotConnector	robot;

	ofstream	record;
	record.open("../data/robot.txt");

	if( !robot.Connect(Create_Comport) )
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");


	while(true)
	{
		robot.ReadData(robotData);
		
		cout << "Charging State		: " << robotData.chargingSate << endl;
		cout << "Voltage	   		: " << robotData.voltage << endl;
		cout << "Current			: " << robotData.current << endl;
		cout << "Battery Temperature: " << robotData.battTemp << endl;
		cout << "Battery Charge		: " << robotData.battCharge << endl;
		cout << "Battery Capacity	: " << robotData.battCap << endl;
		
		cvWaitKey(1000);
	}

	robot.Disconnect();

	return 0;
}