
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
#define ROBOT_SIZE 40
#define MAP_SIZE_X 500
#define MAP_SIZE_Y 400
#define MAP_ROBOT_X 300
#define MAP_ROBOT_Y 200
#define GRID_SIZE 50
#define RESCALE 1

bool lock = false;
int updatePose = 2;

using namespace std;
using namespace cv;
using easywsclient::WebSocket;

KinectConnector kin;
CreateData	robotData;
RobotConnector	robot;
Mat depthImg;
char mode;
Mat colorImg;
Mat indexImg;
Mat pointImg;
queue <pair<int, int> > q;
int white[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
int red[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
int d[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
pair<int, int> p[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
bool visit[MAP_SIZE_X + 100][MAP_SIZE_Y + 100];
double vl, vr;
double posx = -1000, posy = -1000, posz, angle;
int goto_x, goto_y;
int state = 0;
vector<double> robustx, robusty, robustang;

boolean finish = 0;
WebSocket::pointer wp;

double roundDown(double value) {
	return value < 0.0001 && value > -0.0001 ? 0 : value;
}

void convert_to_world_frame(double posx, double posy, double angle, double end_x_robot, double end_y_robot, double& end_world_x, double& end_world_y) {
	end_world_x = roundDown(posx + (end_x_robot * cos(angle) + end_y_robot * sin(angle)));
	end_world_y = roundDown(posy + (end_x_robot * -1 * sin(angle) + end_y_robot * cos(angle)));
}

void plot_score_map(boolean save = false) {

	Mat map(MAP_SIZE_Y / RESCALE, MAP_SIZE_X / RESCALE, CV_8UC3, Scalar(30, 30, 30));

	for (int i = 0; i < MAP_SIZE_X/ RESCALE; i++) {
		for (int j = 0; j < MAP_SIZE_Y/ RESCALE; j++) {
			int sumred = 0, sumwhite = 0;
			for (int k = 0; k < RESCALE; k++)
				for (int l = 0; l < RESCALE; l++) {
					sumred += red[i * RESCALE  + k][j * RESCALE + l];
					sumwhite += white[i * RESCALE + k][j * RESCALE + l];
				}
			if (sumred *10 - sumwhite > 0)
				map.at<Vec3b>(j, i) = Vec3b(0, 0, 255);
			else if (sumwhite > 0)
				map.at<Vec3b>(j, i) = Vec3b(255, 255, 255);
		}
	}

	if (save)
		imwrite("map.jpg", map);
	else {

		int ux = posx + MAP_SIZE_X / 2;
		int uy = posy + MAP_SIZE_Y / 2;
		circle(map, Point(ux/ RESCALE, uy/ RESCALE), ROBOT_SIZE / 2 / RESCALE, Scalar(0, 255, 0), 3);

		double end_x, end_y;
		convert_to_world_frame(ux, uy, angle, 30, 0, end_x, end_y);
		line(map, Point(ux/ RESCALE, uy/ RESCALE), Point(end_x/ RESCALE, end_y/ RESCALE), Scalar(255, 0, 0), 3);
	}

	resize(map, map, Size(MAP_SIZE_X, MAP_SIZE_Y));
	imshow("world", map);
}
bool robot_can_stay_at(int vx, int vy) {
	for (int i = vx - (ROBOT_SIZE - 1) / 2; i < vx + (ROBOT_SIZE + 1) / 2; i++) {
		for (int j = vy - (ROBOT_SIZE - 1) / 2; j < vy + (ROBOT_SIZE + 1) / 2; j++) {

			if (abs(i - MAP_SIZE_X / 2) >(MAP_ROBOT_X / 2) || abs(j - MAP_SIZE_Y / 2) >(MAP_ROBOT_Y / 2))
				continue;

			if (red[i][j] > 0) {
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

			/*if ((red[x][y] * 10 - white[x][y]) > 0) {
				break;
			}*/

			if (n == 1)
				red[x][y] += 1;
			else
				white[x][y] += 1;
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


void updateMap() {


	double tempx, tempy, tempangle;
	tempx = posx;
	tempy = posy;
	tempangle = angle;

	kin.GrabData(depthImg, colorImg, indexImg, pointImg);

	//cout << "update " << tempx << " " << tempy << " " << (tempangle * 180 / M_PI) << endl;

	for (int i = 0; i < 620; i++) {
		///////////////////////////////////////////////
		// find point in robot frame from depth
		///////////////////////////////////////////////
		double end_x_robot = depthImg.at<USHORT>(240, i) / 10.0;
		if (end_x_robot == 0)
			continue;
		double end_y_robot = (i - 320.0) / 531.15 * end_x_robot;

		///////////////////////////////////////////////
		// convert point in robot frame to world frame
		///////////////////////////////////////////////
		double end_x_world, end_y_world;
		convert_to_world_frame(tempx, tempy, tempangle, end_x_robot, end_y_robot, end_x_world, end_y_world);

		///////////////////////////////////////////////
		// update score from point
		///////////////////////////////////////////////
		update_score(tempx + MAP_SIZE_X / 2, tempy + MAP_SIZE_Y / 2, end_x_world + MAP_SIZE_X / 2, end_y_world + MAP_SIZE_Y / 2);

	}

	for (int i = tempx + MAP_SIZE_X / 2 - (ROBOT_SIZE - 1) / 2; i < tempx + MAP_SIZE_X / 2 + (ROBOT_SIZE + 1) / 2; i++) {
		for (int j = tempy + MAP_SIZE_Y / 2 - (ROBOT_SIZE - 1) / 2; j < tempy + MAP_SIZE_Y / 2 + (ROBOT_SIZE + 1) / 2; j++) {
			if (i < 0 || i >= MAP_ROBOT_X || j < 0 || j >= MAP_ROBOT_Y)
				continue;

			white[i][j] += 1000;
		}
	}

	plot_score_map(false);
}
double min(double x, double y) {
	return x < y ? x : y;
}

void walk_to(int endx, int endy) {

	cvNamedWindow("robot");


	double diffx = endx - posx;
	double diffy = posy - endy;

	if (state == 1) {
		if (!robot_can_stay_at(goto_x, goto_y)) {
			state = 0;
			return;
		}

		double target_angle = atan(diffy / diffx);
		if (endx < posx)
			target_angle += M_PI;

		target_angle = (target_angle + 2 * M_PI);
		while (target_angle > M_PI * 2) {
			target_angle -= M_PI * 2;
		}
		angle += 2 * M_PI;
		while (angle > M_PI * 2) {
			angle -= M_PI * 2;
		}

		double diff;
		if (target_angle > angle)
			diff = min(target_angle - angle, angle + (2 * M_PI - target_angle));
		else
			diff = min(angle - target_angle, target_angle + (2 * M_PI - angle));

		if ((diff * 180 / M_PI) > 15) {

			if (target_angle > angle) {
				if (target_angle - angle < angle + (2 * M_PI - target_angle)) {
					vr = 0.3;
					vl = -0.3;
				}
				else {
					vr = -0.3;
					vl = 0.3;
				}
			}
			else {
				if (angle - target_angle < target_angle + (2 * M_PI - angle)) {
					vr = -0.3;
					vl = 0.3;
				}
				else {
					vr = 0.3;
					vl = -0.3;
				}
			}

			int velL = (int)(vl*Create_MaxVel);
			int velR = (int)(vr*Create_MaxVel);

			robot.DriveDirect(velL, velR);
			Sleep(500);
			robot.DriveDirect(0, 0);
			Sleep(100);
		}
		else if (sqrt(diffx * diffx + diffy * diffy) > 25) {
			vl = 0.5;
			vr = 0.5;

			int velL = (int)(vl*Create_MaxVel);
			int velR = (int)(vr*Create_MaxVel);

			robot.DriveDirect(velL, velR);
			Sleep(800);
			robot.DriveDirect(0, 0);
			Sleep(100);
		}
		else {
			 if (visit[goto_x / GRID_SIZE][goto_y / GRID_SIZE])
				 state = 0;
			 else state = 2;
		}
	}
	else if (state >= 2) {
		vl = -0.5;
		vr = 0.5;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		robot.DriveDirect(velL, velR);
		Sleep(700);
		robot.DriveDirect(0, 0);
		Sleep(100);

		state++;
		if (state >= 6) {
			visit[goto_x / GRID_SIZE][goto_y / GRID_SIZE] = true;
			state = 0;
		}
	}
}



boolean get_next_point(int posx, int posy, int& des_x, int& des_y) {

	int dx[] = { 0, GRID_SIZE,0,-1 * GRID_SIZE };
	int dy[] = { -1 * GRID_SIZE,0,GRID_SIZE,0 };

	while (!q.empty())
		q.pop();
	for (int i = 0; i <= MAP_SIZE_X; i++)
		for (int j = 0; j <= MAP_SIZE_Y; j++)
			d[i][j] = 0;

	int startx = ((floor)(posx / GRID_SIZE))*GRID_SIZE + GRID_SIZE / 2.0;
	int starty = ((floor)(posy / GRID_SIZE))*GRID_SIZE + GRID_SIZE / 2.0;
	q.push({ startx, starty });
	d[startx][starty] = 1;

	while (!q.empty()) {

		int ux = q.front().first;
		int uy = q.front().second;
		q.pop();
		for (int i = 0; i<4; i++) {
			int vx = ux + dx[i];
			int vy = uy + dy[i];

			if (abs(vx - MAP_SIZE_X / 2) < (MAP_ROBOT_X / 2) && vx >= 0 && abs(vy - MAP_SIZE_Y / 2) < (MAP_ROBOT_Y / 2) && vy >= 0 && d[vx][vy] == 0 && robot_can_stay_at(vx, vy)) {
				d[vx][vy] = d[ux][uy] + 1;
				p[vx][vy] = make_pair(ux, uy);
				q.push(make_pair(vx, vy));

				if (!visit[vx / GRID_SIZE][vy / GRID_SIZE]) {
					des_x = vx;
					des_y = vy;

					while (des_x != startx || des_y != starty) {

						if (p[des_x][des_y].first == startx && p[des_x][des_y].second == starty)
							return true;

						int keepx = p[des_x][des_y].first;
						int keepy = p[des_x][des_y].second;
						des_x = keepx;
						des_y = keepy;
					}

				}
			}
		}
	}

	return false;
}

void get_pose(const std::string & message) {

	cout << "answer" << endl;
	string res = message.c_str();
	res = res.substr(3, res.size() - 3);
	while (res.find("<br/>") != string::npos) {

		if (res.find("id: 8") == 0) {
			cout << "found" << endl;
			res = res.substr(0, res.find("<br/>"));

			///////////////////////////////////////////////
			// extract position and angle from response
			///////////////////////////////////////////////
			double tempx, tempy, tempz, tempangle;
			tempx = atof(res.substr(res.find("pos:") + 5, res.find(",") - res.find("pos:") - 5).c_str());
			res = res.substr(res.find(",") + 2);
			tempy = atof(res.substr(0, res.find(",")).c_str());
			res = res.substr(res.find(",") + 2);
			tempz = atof(res.substr(0, res.find(",")).c_str());
			res = res.substr(res.find("angle: ") + 7);

			tempangle = 180 + atof(res.c_str());
			tempangle = tempangle * M_PI / 180;


			if (updatePose > 0 || (posx == -1000 && posy == -1000)) {
				updatePose--;
				double qx, qy;
				convert_to_world_frame(tempx, tempy, tempangle, 7, 0, qx, qy);

				robustx.push_back(qx);
				robusty.push_back(qy);
				robustang.push_back(tempangle);
				//angle = M_PI / 2;
				//posx = ((int)posx / RESCALE) * RESCALE;
				//posy = ((int)posy / RESCALE) * RESCALE;
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
	wp = WebSocket::from_url("ws://192.168.1.59:8080/pose");
	return true;
}

void close() {
	//robot.Disconnect();
	wp->close();
	delete wp;
	WSACleanup();
}


void walk() {

	///////////////////////////////////////////////
	// Meen: find path to grid that score 0 (BFS)
	///////////////////////////////////////////////
	if (state == 0) {
		if (!get_next_point(posx + MAP_SIZE_X / 2, posy + MAP_SIZE_Y / 2,goto_x, goto_y)) {
			finish = true;
		}

		state = 1;
	}

	double des_x = goto_x - MAP_SIZE_X / 2.0;
	double des_y = goto_y - MAP_SIZE_Y / 2.0;
	cout << "goto " << posx << " " << posy << " " << des_x << " " << des_y << "state " << state << endl;
	walk_to(des_x, des_y);
}

int main()
{
	cvNamedWindow("robot");
	//freopen("output.txt", "w", stdout);


	cout << "Press A for autonomous mode" << endl;
	cout << "Press other for hand mode" << endl;
	cout << "Choose : ";
	cin >> mode;

	if (initial_robot() && initial_kinect() && initial_socket()) {

		updatePose = 5;

		cout << "Start" << endl;
		finish = false;
		while (true)
		{
			cout << "poll " << endl;
			lock = true;
			wp->poll();
			wp->send("");
			wp->dispatch(get_pose);

			cvWaitKey(100);

			if (lock)
				continue;

			if (updatePose <= 0) {
				sort(robustx.begin(), robustx.end());
				sort(robusty.begin(), robusty.end());
				sort(robustang.begin(), robustang.end());

				posx = robustx[robustx.size() / 2];
				posy = robusty[robusty.size() / 2];
				angle = robustang[robustang.size() / 2];

				robustx.clear();
				robusty.clear();
				robustang.clear();


				updateMap();

				if (mode == 'a') {

					walk();

					for (int i = 0; i < 10; i++) {
						for (int j = 0; j < 10; j++) {
							int x = i*GRID_SIZE + GRID_SIZE / 2.0;
							int y = j*GRID_SIZE + GRID_SIZE / 2.0;
							if (!robot_can_stay_at(x, y))
								cout << "2" << " ";
							else
								cout << visit[i][j] << " ";
						}
						cout << endl;
					}
				}

				updatePose = 5;
			}


			if (finish)
				break;
		}

		plot_score_map(true);

		cout << "end" << endl;
		close();
	}

	getchar();

	return 0;
}

