
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
int score[1000][1000];

void update_score(int sx, int sy, int ex, int ey) {
	int length = (int)floor(sqrt((ex - sx)*(ex - sx) + (ey - sy)*(ey - sy)));

	double k = 1 / (double)length;

	for (int i = 0; i < length; i++) {
		int x, y;
		x = sx + k*(i + 1)*(ex - sx);
		y = sy + k*(i + 1)*(ey - sy);
		if (i > (length - 2)) {
			score[x][y] += 1;
			score[x][y + 1] += 1;
			score[x + 1][y] += 1;
			score[x + 1][y + 1] += 1;
		}
		else {
			score[x][y] += -1;
			score[x][y + 1] += -1;
			score[x + 1][y] += -1;
			score[x + 1][y + 1] += -1;
		}
	}

	score[ex][ey] += 2;
}

void convert_to_world_frame(double posx, double posy, double angle, double end_robot_x, double end_robot_y, double& end_world_x, double& end_world_y) {
	// do something


	// end_world_x = ...;
	// end_world_y = ...;
}

void walk(double vl, double vr) {

	int velL = (int)(vl*Create_MaxVel);
	int velR = (int)(vr*Create_MaxVel);

	robot.DriveDirect(velL, velR);
	Sleep(100);
	robot.DriveDirect(0, 0);
}

void walk_to(double posx,double posy, double angle, int endx, int endy) {
	double diffx = abs(posx - endx);
	double diffy = abs(posy - endy);
	
	double target_angle = atan(diffy / diffx);
	double diff_angle = target_angle - angle;
	
	if (diff_angle > 5) {
		double r = /*diff_angle <= 180 ? -1 :*/ 1;
		double vl, vr;
		vl = r;
		vr = r;
		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		
		cvWaitKey(20);
		return;
	}

	if( diffx > 1 && diffy > 1 )
	{
	
		double vx, vz;
		vx = vz = 0.0;
			
		vx = 1.0;

		double vl = vx - vz;
		double vr = vx + vz;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		cvWaitKey(20);
		return;
	}

}

void handle_response(const std::string & message)
{
	//	kin.GrabData(depthImg, colorImg, indexImg, pointImg);
	//imshow("depthImg", depthImg);
	//imshow("colorImg", colorImg);

	string res = message.c_str();
	res = res.substr(3, res.size() - 3);

	while (res.find("<br/>") != string::npos) {
		///////////////////////////////////////////////
		// find my robot in response
		///////////////////////////////////////////////
		if (res.find("id: 8") == 0) {
			res = res.substr(0, res.find("<br/>"));
			
			///////////////////////////////////////////////
			// extract position and angle from response
			///////////////////////////////////////////////
			double posx, posy, posz;
			double angle;
			posx = atof(res.substr(res.find("pos:") + 5, res.find(",") - res.find("pos:") - 5).c_str());
			res = res.substr(res.find(",") + 2);
			posy = atof(res.substr(0, res.find(",")).c_str());
			res = res.substr(res.find(",") + 2);
			posz = atof(res.substr(0, res.find(",")).c_str());
			res = res.substr(res.find("angle: ") + 7);

			double neg = 1;
			if (res[0] == '-') {
				neg = -1;
				res = res.substr(1);
			}
			angle = neg * atof(res.c_str());
			
			walk_to(posx,posy,angle,165,140);

			//cout << posx << " " << posy <<" " <<posz <<" " <<angle  << " end\n";

			for (int i = 0; i < 640; i++) {
				///////////////////////////////////////////////
				// find point in robot frame from depth
				///////////////////////////////////////////////
				double end_y_robot = depthImg.at<USHORT>(i, 240);
				double end_x_robot = (i - 320.0) / 575.0 * end_y_robot;

				///////////////////////////////////////////////
				// convert point in robot frame to world frame
				///////////////////////////////////////////////
				double end_x_world, end_y_world;
				convert_to_world_frame(posx, posy, angle, end_x_robot, end_y_robot, end_x_world , end_y_world);

				///////////////////////////////////////////////
				// update score from point
				///////////////////////////////////////////////
				update_score(posx, posy, end_x_world, end_y_world);
			}

			///////////////////////////////////////////////
			// Boss: find path to grid that score 0 (BFS ??)
			///////////////////////////////////////////////
			// ---

			///////////////////////////////////////////////
			// walk to first point of Boss's path
			///////////////////////////////////////////////
			// ---

		}
		else 
			res = res.substr(res.find("<br/>") + 5, res.size() - res.find("<br/>") - 5);
	}
	cout << "Sasad\n";
}

int main()
{
	cvNamedWindow("Robot");

	///////////////////////////////////////////////
	// initialize robot
	///////////////////////////////////////////////
	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}
	robot.DriveDirect(0, 0);
	
	///////////////////////////////////////////////
	// initialize kinect
	///////////////////////////////////////////////
	//kin = KinectConnector();
	//if (!kin.Connect()) {
	//	cout << "Error : Can't connect to kinect" << endl;
	//	return 1;
	//}


	///////////////////////////////////////////////
	// initialize socket
	///////////////////////////////////////////////
	INT rc;
	WSADATA wsaData;
	rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (rc) {
		printf("WSAStartup Failed.\n");
		return 1;
	}
	using easywsclient::WebSocket;
	WebSocket::pointer wp = WebSocket::from_url("ws://192.168.1.59:8080/pose");

	///////////////////////////////////////////////
	// slam
	///////////////////////////////////////////////
	while (true)
	{
		wp->poll();
		wp->send("");
		wp->dispatch(handle_response);
		
		cvWaitKey(100);
	}

	///////////////////////////////////////////////
	// close everything;
	///////////////////////////////////////////////
	//robot.Disconnect();
	wp->close();
	delete wp;
	WSACleanup();

	return 0;
}


