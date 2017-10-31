
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <winsock2.h>
#include <io.h>
#include <ws2tcpip.h>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <KinectConnector.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "easywsclient.cpp" 

#define Create_Comport "COM3"

bool isRecord = false;

using namespace std;
using namespace cv;

KinectConnector kin;
CreateData	robotData;
RobotConnector	robot;
Mat depthImg;
Mat colorImg;
Mat indexImg;
Mat pointImg;

void handle_message(const std::string & message)
{

//	kin.GrabData(depthImg, colorImg, indexImg, pointImg);
	//imshow("depthImg", depthImg);
	//imshow("colorImg", colorImg);

	string res = message.c_str();
	cout << res << endl;
	//res = res.substr(3, res.size() - 3);

	//while (res.find("<br/>") != string::npos) {
	//	if (res.find("id: 8") == 0) {
	//		res = res.substr(0, res.find("<br/>"));
	//		
	//		double posx, posy, posz;
	//		double angle;
	//		posx = atof(res.substr(res.find("pos:") + 5, res.find(",") - res.find("pos:") - 5).c_str());
	//		res = res.substr(res.find(",") + 2);
	//		posy = atof(res.substr(0, res.find(",")).c_str());
	//		res = res.substr(res.find(",") + 2);
	//		posz = atof(res.substr(0, res.find(",")).c_str());
	//		res = res.substr(res.find("angle: ") + 8);
	//		double neg = 1;
	//		if (res[0] == '-') {
	//			neg = -1;
	//			res = res.substr(1);
	//		}
	//		angle = neg * atof(res.c_str());

	//		cout << posx << " " << posy <<" " <<posz <<" " <<angle  << " end\n";

	//		break;
	//	}
	//	else 
	//		res = res.substr(res.find("<br/>") + 5, res.size() - res.find("<br/>") - 5);
	//}
	//cout << "Sasad\n";
}

int main()
{

	// initialize robot
/*
	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	//robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");
*/

	// initialize kinect
	kin = KinectConnector();
	if (!kin.Connect()) {
		cout << "Error : Can't connect to kinect" << endl;
		return 1;
	}

	// initialize socket
	INT rc;
	WSADATA wsaData;
	rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (rc) {
		printf("WSAStartup Failed.\n");
		return 1;
	}
	using easywsclient::WebSocket;
	WebSocket::pointer wp = WebSocket::from_url("ws://192.168.1.59:8080/pose");


	while (true)
	{
		wp->poll();
		wp->send("");
		wp->dispatch(handle_message);
		

		//char c = cvWaitKey(30);
		//if (c == 27) break;

		//double vx, vz;
		//vx = vz = 0.0;

		//switch (c)
		//{
		//case 'w': vx = +1; break;
		//case 's': vx = -1; break;
		//case 'a': vz = +1;  break;
		//case 'd': vz = -1;  break;
		//case ' ': vx = vz = 0; break;
		//case 'c': robot.Connect(Create_Comport); break;
		//}

		//double vl = vx - vz;
		//double vr = vx + vz;


		//int velL = (int)(vl*Create_MaxVel);
		//int velR = (int)(vr*Create_MaxVel);

		//robot.DriveDirect(velL, velR);
		//robot.ReadData(robotData);

		cvWaitKey(100);
	}

	// close everything;
	
	//robot.Disconnect();
	wp->close();
	delete wp;
	WSACleanup();

	return 0;
}


