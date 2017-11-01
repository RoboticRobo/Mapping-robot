
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
#define M_PI 3.141592653589793238462643383279502884L

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
int score[1100][1100];

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

double roundDown(double value) {
	return value < 0.0001 && value > -0.0001 ? 0 : value;
}

void convert_to_world_frame(double posx, double posy, double angle, double end_x_robot, double end_y_robot, double& end_world_x, double& end_world_y) {

	
	end_world_x = roundDown(posx + (end_x_robot * sin(angle) + end_y_robot * cos(M_PI + angle)));
	end_world_y = roundDown(posy + (end_x_robot * cos(angle) + end_y_robot * sin(M_PI + angle)));
}

//void walk(double vl, double vr) {
//
//	int velL = (int)(vl*Create_MaxVel);
//	int velR = (int)(vr*Create_MaxVel);
//
//	robot.DriveDirect(velL, velR);
//	Sleep(100);
//	robot.DriveDirect(0, 0);
//}

void plot_score_map() {
	Mat map(1000, 1000, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i < 1000; i++) {
		for (int j = 0; j < 1000; j++) {
			if(score[j][i] > 0)
				map.at<Vec3b>(j, i) = Vec3b(0, 0, 255);
		}
	}
	imshow("world", map);
}

void handle_response(const std::string & message) {

	kin.GrabData(depthImg, colorImg, indexImg, pointImg);
	imshow("depthImg", depthImg);
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

			//cout << posx << " " << posy <<" " <<posz <<" " <<angle  << " end\n";

			for (int i = 0; i < 640; i++) {
				///////////////////////////////////////////////
				// find point in robot frame from depth
				///////////////////////////////////////////////
				double end_y_robot = depthImg.at<USHORT>(i, 240) / 10.0;
				if (end_y_robot == 0)
					continue;
				double end_x_robot = (i - 320.0) / 575.0 * end_y_robot;

				///////////////////////////////////////////////
				// convert point in robot frame to world frame
				///////////////////////////////////////////////
				double end_x_world, end_y_world;
				convert_to_world_frame(posx, posy, angle, end_x_robot, end_y_robot, end_x_world , end_y_world);


				///////////////////////////////////////////////
				// update score from point
				///////////////////////////////////////////////
				if (end_x_world > 500 || end_x_world < -500 || end_y_world > 500 || end_y_world < -500) {
					cout << "posx = " << posx << endl;
					cout << "posy = " << posy << endl;
					cout << "angle = " << angle << endl;
					cout << "end_x_robot = " << end_x_robot << endl;
					cout << "end_y_robot = " << end_y_robot << endl;
					cout << "end_x_world = " << end_x_world << endl;
					cout << "end_y_world = " << end_y_world << endl;
					cin >> posx;

				}	
				update_score(posx + 500, posy + 500, end_x_world + 500, end_y_world + 500);


			}

			plot_score_map();

			///////////////////////////////////////////////
			// Meen: find path to grid that score 0 (BFS ??)
			///////////////////////////////////////////////
			// ---

			///////////////////////////////////////////////
			// walk to first point of Meen's path
			///////////////////////////////////////////////
			// ---

		}
		else 
			res = res.substr(res.find("<br/>") + 5, res.size() - res.find("<br/>") - 5);
	}
}

int main()
{
	cvNamedWindow("Robot");

	///////////////////////////////////////////////
	// initialize robot
	///////////////////////////////////////////////
	/*
	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}
	//robot.DriveDirect(0, 0);
	*/

	///////////////////////////////////////////////
	// initialize kinect
	///////////////////////////////////////////////
	kin = KinectConnector();
	if (!kin.Connect()) {
		cout << "Error : Can't connect to kinect" << endl;
		return 1;
	}

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


