
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <winsock2.h>
#include <io.h>
#include <ws2tcpip.h>
#include <queue>
#include <stack>

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
#define ROBOT_SIZE 50
#define MAP_SIZE_X 500
#define MAP_SIZE_Y 370
#define MAP_ROBOT_X 380
#define MAP_ROBOT_Y 285

bool lock = false;

using namespace std;
using namespace cv;
using easywsclient::WebSocket;

KinectConnector kin;
CreateData	robotData;
RobotConnector	robot;
Mat depthImg;
Mat colorImg;
Mat indexImg;
Mat pointImg;
queue <pair<int, int> > q;
double score[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
int d[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
pair<int, int> p[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
double vl, vr;

boolean finish = 0;
WebSocket::pointer wp;

bool robot_can_stay_at(int vx, int vy) {
	for (int i = vx - (ROBOT_SIZE - 1) / 2; i < vx + (ROBOT_SIZE + 1) / 2; i++) {
		for (int j = vy - (ROBOT_SIZE - 1) / 2; j < vy + (ROBOT_SIZE + 1) / 2; j++) {
			if (i < 0 || i >= MAP_ROBOT_X || j < 0 || j >= MAP_ROBOT_Y)
				continue;

			if (score[i][j] > 0.7) {
				return false;
			}
		}
	}
	return true;
}

void update_score(int sx, int sy, int ex, int ey) {

	int dx = abs(ex - sx);
	int dy = abs(ey - sy);
	int x = sx;
	int y = sy;
	int n = 1 + dx + dy;
	int x_inc = (ex > sx) ? 1 : -1;
	int y_inc = (ey > sy) ? 1 : -1;
	int error = dx - dy;
	dx *= 2;
	dy *= 2;

	for (; n > 0; --n)
	{
		if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
			if (n == 1 || score[x][y] > 0.8) {
				score[x][y] *= 1.05;
				if (score[x][y] >= 0.9)
					score[x][y] = 0.9;
			}
			else {
				if (score[x][y] < 0.9) {
					score[x][y] *= 0.98;
					if (score[x][y] <= 0.1)
						score[x][y] = 0.1;
				}
			}

			if (score[x][y] > 0.8)
				break;
		}

		if (error > 0) {
			x += x_inc;
			error -= dy;
		}
		else {
			y += y_inc;
			error += dx;
		}
	}
}


void walk_to(double posx, double posy, double angle, int endx, int endy) {

	double diffx = posx - endx;
	double diffy = endy - posy;

	double target_angle = atan(diffx / diffy);
	if (posy > endy)
		target_angle += M_PI;

	target_angle = (target_angle + 2*M_PI);
	while (target_angle > M_PI * 2) {
		target_angle -= M_PI * 2;
	}

	double diff_angle = target_angle - angle;
	cout << target_angle * 180 / M_PI << " " << angle * 180 / M_PI << " " << diff_angle * 180 / M_PI << endl;

	if (abs(diff_angle * 180 / M_PI) > 10) {

		vr = 0.1;
		vl = -0.1;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		return;
	}

	if (diffx > 1 || diffy > 1)
	{
		vl = 0.1;
		vr = 0.1;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		return;
	}

}


double roundDown(double value) {
	return value < 0.0001 && value > -0.0001 ? 0 : value;
}

void convert_to_world_frame(double posx, double posy, double angle, double end_x_robot, double end_y_robot, double& end_world_x, double& end_world_y) {
	end_world_x = roundDown(posx + (end_x_robot * cos(angle) + end_y_robot * sin(angle)));
	end_world_y = roundDown(posy + (end_x_robot * -1 * sin(angle) + end_y_robot * cos(angle)));
}

void plot_score_map(boolean save = false, int posx = -1, int posy = -1, double angle = 0) {

	Mat map(MAP_SIZE_Y, MAP_SIZE_X, CV_8UC3, Scalar(30, 30, 30));

	for (int i = 0; i < MAP_SIZE_X; i++) {
		for (int j = 0; j < MAP_SIZE_Y; j++) {
			if (score[i][j] < 0.4)
				map.at<Vec3b>(j, i) = Vec3b(255, 255, 255);
			if (score[i][j] > 0.8)
				map.at<Vec3b>(j, i) = Vec3b(0, 0, 255);
		}
	}

	if (posx != -1 && posy != -1) {
		for (int i = posx - (ROBOT_SIZE - 1) / 2; i < posx + (ROBOT_SIZE + 1) / 2; i++) {
			for (int j = posy - (ROBOT_SIZE - 1) / 2; j < posy + (ROBOT_SIZE + 1) / 2; j++) {
				if(i>=0 && i< MAP_SIZE_X && j>=0 && j< MAP_SIZE_Y)
					map.at<Vec3b>(j, i) = Vec3b(0, 255, 0);
			}
		}

		double end_x, end_y;
		convert_to_world_frame(posx, posy, angle, 20, 0, end_x, end_y);
		line(map, Point(posx, posy), Point(end_x, end_y), Scalar(255, 0, 0), 3);
	}

	imshow("world", map);
	
	if(save)
		imwrite("map.jpg", map);

}

boolean get_next_point(int posx, int posy, int& des_x, int& des_y) {
	int dx[] = { 1,0,0,-1 };
	int dy[] = { 0,1,-1,0 };

	while (!q.empty())
		q.pop();
	for (int i = 0; i <= MAP_SIZE_X; i++)
		for (int j = 0; j <= MAP_SIZE_Y; j++)
			d[i][j] = 0;

	q.push(make_pair(posx, posy));
	d[posx][posy] = 1;

	cout << "bfs" << endl;
	while (!q.empty()) {

		int ux = q.front().first;
		int uy = q.front().second;
		q.pop();
		for (int i = 0; i<4; i++) {
			int vx = ux + dx[i];
			int vy = uy + dy[i];

			if (vx < MAP_SIZE_X && vx >= 0 && vy < MAP_SIZE_Y && vy >= 0 && d[vx][vy] == 0 && robot_can_stay_at(vx, vy)) {
				d[vx][vy] = d[ux][uy] + 1;
				p[vx][vy] = make_pair(ux, uy);
				q.push(make_pair(vx, vy));

				if (score[vx][vy] >= 0.3 && score[vx][vy] <= 0.7 && vx < MAP_ROBOT_X && vx >= 0 && vy < MAP_ROBOT_Y && vy >= 0) {
					des_x = vx;
					des_y = vy;

//					cout << posx <<" " <<posy <<" " << vx << " " << vy << endl;
					while (des_x != posx || des_y != posy) {
	//					cout << des_x << " " << des_y << " | " << p[des_x][des_y].first << " " << p[des_x][des_y].second << endl;
						if (p[des_x][des_y].first == posx && p[des_x][des_y].second == posy)
							return true;

						int keepx = p[des_x][des_y].first;
						int keepy = p[des_x][des_y].second;
						des_x = keepx;
						des_y = keepy;
					}
//					cout << "gg" << endl;
					
				}
			}
		}
	}

	return false;
}

void handle_response(const std::string & message) {

	cout << "test" << endl;
	cvNamedWindow("robot");
	kin.GrabData(depthImg, colorImg, indexImg, pointImg);

//	imshow("depthImg", depthImg);
//	imshow("colorImg", colorImg);

	string res = message.c_str();
	res = res.substr(3, res.size() - 3);
	while (res.find("<br/>") != string::npos) {

		if (res.find("id: 8") == 0) {
			cout << "found" << endl;
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
			angle = 180 + neg * atof(res.c_str());
			//cout << posx << " " << posy <<" " <<posz <<" " <<angle  << " end\n";
			angle = angle * M_PI / 180;
			double tempx, tempy;
			convert_to_world_frame(posx, posy, angle, 13, 0, tempx, tempy);
			posx = tempx;
			posy = tempy;

			for (int i = 0; i < 640; i++) {

				//cout << i << endl;
				///////////////////////////////////////////////
				// find point in robot frame from depth
				///////////////////////////////////////////////
				double end_x_robot = depthImg.at<USHORT>(240, i) / 10.0;
				if (end_x_robot == 0)
					continue;
				double end_y_robot = (i - 320.0) / 575	 * end_x_robot;
				//cout << "cal" << endl;

				///////////////////////////////////////////////
				// convert point in robot frame to world frame
				///////////////////////////////////////////////
				double end_x_world, end_y_world;
				convert_to_world_frame(posx, posy, angle, end_x_robot, end_y_robot, end_x_world, end_y_world);
				//cout << "convert" << endl;

				///////////////////////////////////////////////
				// update score from point
				///////////////////////////////////////////////
				
				//cout << "posx = " << posx << endl;
				//cout << "posy = " << posy << endl;
				//cout << "angle = " << angle << endl;
				//cout << "end_x_robot = " << end_x_robot << endl;
				//cout << "end_y_robot = " << end_y_robot << endl;
				//cout << "end_x_world = " << end_x_world << endl;
				//cout << "end_y_world = " << end_y_world << endl;
				update_score(posx + MAP_SIZE_X / 2, posy + MAP_SIZE_Y / 2, end_x_world + MAP_SIZE_X / 2, end_y_world + MAP_SIZE_Y / 2);

				//cout << "updated" << endl;

			}

			plot_score_map(false, posx + MAP_SIZE_X/2, posy + MAP_SIZE_Y / 2, angle);

			///////////////////////////////////////////////
			// Meen: find path to grid that score 0 (BFS)
			///////////////////////////////////////////////
			/*int des_x, des_y;
			if (!get_next_point(posx + MAP_SIZE_X/ 2, posy + MAP_SIZE_Y / 2, des_x, des_y)) {
				finish = true;
			}
			else {
				des_x -= MAP_SIZE_X / 2;
				des_y -= MAP_SIZE_Y / 2;
				cout << "kuy " << endl;
				cout << posx << " " << posy << " " << des_x << " " << des_y << endl;
				walk_to(posx, posy, angle, des_x, des_y);
			}*/


			char c = cvWaitKey(30);

			switch (c)
			{
			case 'w': vl = 0.2; vr = 0.2; break;
			case 's': vl = -0.2; vr = -0.2; break;
			case 'a': vl = -0.2; vr = 0.2; break;
			case 'd': vl = 0.2; vr = -0.2; break;
			}
			

			int velL = (int)(vl*Create_MaxVel);
			int velR = (int)(vr*Create_MaxVel);

			robot.DriveDirect(velL, velR);

			vl /= 1.2;
			vr /= 1.2;
			if (abs(vl) < 0.08 || abs(vr) < 0.08) {
				vl = 0;
				vr = 0;
			}
		}
		else
			res = res.substr(res.find("<br/>") + 5, res.size() - res.find("<br/>") - 5);
	}

	lock = false;
}

boolean initial_kinect() {
	kin = KinectConnector();
	if (!kin.Connect()) {
		cout << "Error : Can't connect to kinect" << endl;
		return false;
	}
	kin.GrabData(depthImg, colorImg, indexImg, pointImg);
	return true;
}

boolean initial_robot() {
	if (!robot.Connect(Create_Comport)) {
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return false;
	}
	robot.DriveDirect(0, 0);
	return true;
}

boolean initial_socket() {
	INT rc;
	WSADATA wsaData;
	rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (rc) {
		printf("WSAStartup Failed.\n");
		return false;
	}
	wp = WebSocket::from_url("ws://192.168.1.59:8081/pose");
	return true;
}

void close() {
	//robot.Disconnect();
	wp->close();
	delete wp;
	WSACleanup();
}

int main()
{
	cvNamedWindow("robot");

	if(initial_robot() && initial_kinect() && initial_socket()) {

		for (int i = 0; i < MAP_SIZE_X; i++) {
			for (int j = 0; j < MAP_SIZE_Y; j++)
				score[i][j] = 0.5;
		}

		cout << "Start" << endl;
		finish = false;
		while (true)
		{				
			lock = true;
			wp->poll();
			wp->send("");
			wp->dispatch(handle_response);


			if (finish)
				break;

			cvWaitKey(100);
		}

		plot_score_map(true);

		cout << "end" << endl;

		cout << score[100][100] << endl;

		close();
	}

	getchar();

	return 0;
}


