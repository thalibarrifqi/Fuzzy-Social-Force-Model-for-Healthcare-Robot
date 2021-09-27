#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <Windows.h>
#include <iostream>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#pragma once

extern "C"
{
#include "extApi.h"
#include "extApiPlatform.h"
}

// Parameters of the Social Force Model - X. Yang (2014)
#define m 20 // robot mass = 50 kg
#define tau 0.1 // robot reaction time (sec)
#define phi 3.142857
#define major_axis 50 // 100 pixels = 1 meters //Depan-belakang
#define minor_axis 50 // 100 pixels = 1 meters //samping kiri-kanan
#define scale 1 // scale pixel into world infos

// object handling
int robothandle;
int personhandle1, personhandle2, personhandle3; // untuk handling human
int obstaclehandle0, obstaclehandle1, obstaclehandle2, obstaclehandle3; // untuk handling dynamic obstacle lainnya (misal: robot)
int obstaclehandle4, obstaclehandle5; // untuk handling dynamic obstacle lainnya (misal: robot)
int obstaclehandle6, obstaclehandle7, obstaclehandle8, obstaclehandle9; // untuk handling dynamic obstacle lainnya (misal: robot)
int goalhandle;
cv::Mat image(500, 500, CV_8UC3);
cv::Mat img;
uchar* laserScannerData1, *laserScannerData2;
int dataSize;
void Scene();
// Fuzzy Inference System
float triange(float a, float b, float c, float x);
float fis(float a, float b);

struct DataFromSimulator
{
	float pos[3]; // world coordinate
	float ori[3]; // world coordinate
	float wl, wr;
	float velx, vely, vtheta;
	cv::Point radii;
	float radius;
	float alpha;
	float dist;
};

struct RepulsiveForce
{
	float force, force1, force2, angle, dist, dist1, dist2, k, rel_angle;
	cv::Point pos;
};

struct AttractiveForce
{
	float force, angle, dist;
	cv::Point pos;
};

struct laser
{
	float x, y, z, dist_from_urg, dist_from_robot_center, angle;
	cv::Point urg, from_robot_center[700], from_urg, obj[200];
	float dist[700], alpha[700], min_dist1, min_dist2, beta1, beta2;
};

struct laser lrf[5];

struct posori
{
	cv::Point pos;
	float ori;
};

struct group
{
	struct posori person[20]; // posori = position and orientation
	struct posori obstacle[20]; // posori = position and orientation
};

// Class for handling robot's properties
class CRobot
{
public:
	DataFromSimulator robot; // real data
	cv::Point robot_pos, robot_ori, pspace; // visualization data
	void interaction_space(int);
	void laser(int);
	RepulsiveForce dynamics(int, struct group, float, float, float); // moving obstacles
	RepulsiveForce intention(); // intention of moving obstacles
	RepulsiveForce statics(float, float, float); // static obstacles
	AttractiveForce goal(int, float, cv::Point, float); // goal
};

// Class for handling person's properties
class CPerson
{
public:
	DataFromSimulator person[20]; // real data
	cv::Point person_pos[20], person_ori[20], pspace[20]; // visualization data
	void interaction_space(int, int);
};

// Class for handling obstacle's properties
class CObstacle
{
public:
	DataFromSimulator obstacle[20]; // real data
	cv::Point obstacle_pos[20], obstacle_ori[20], ospace[20]; // visualization data
	void interaction_space(int, int);
};

// Class for handling goal's properties
class CGoal
{
public:
	DataFromSimulator goal; // real data
	cv::Point goal_pos;
	void interaction_space(int);
};

void CPerson::interaction_space(int clientID, int n_person)
{
	int i, j;
	char buf[5];

	// Retrieve data from the simulator - person 0 position and orientation
	simxGetObjectHandle(clientID, "Bill", &personhandle1, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, personhandle1, -1, person[0].pos, simx_opmode_streaming) == simx_error_noerror)
		//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[0].pos[0],p[0].pos[1],p[0].pos[2]);
	if (simxGetObjectOrientation(clientID, personhandle1, -1, person[0].ori, simx_opmode_streaming) == simx_error_noerror)
		//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[0].ori[0],p[0].ori[1],p[0].ori[2]);

		// Retrieve data from the simulator - person 2 position and orientation
		simxGetObjectHandle(clientID, "Bill#0", &personhandle2, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, personhandle2, -1, person[1].pos, simx_opmode_streaming) == simx_error_noerror)
		//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, personhandle2, -1, person[1].ori, simx_opmode_streaming) == simx_error_noerror)
		//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

		// Retrieve data from the simulator - person 2 position and orientation
		simxGetObjectHandle(clientID, "Bill#1", &personhandle3, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, personhandle3, -1, person[2].pos, simx_opmode_streaming) == simx_error_noerror)
		//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, personhandle3, -1, person[2].ori, simx_opmode_streaming) == simx_error_noerror)
		//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	for (i = 0; i < n_person; i++)
	{
		// Calculate person position from simulator to image
		person_pos[i].x = 50 * person[i].pos[0] + 250; //+ 450;
		person_pos[i].y = 50 * person[i].pos[1] + 250; //+ 300;

		// Calculate person orientation from simulator to image
		person_ori[i].x = person_pos[i].x + 15 * cos(person[i].ori[2]);
		person_ori[i].y = person_pos[i].y + 15 * sin(person[i].ori[2]);

		// Person personal space
		for (j = -180; j < 180; j++)
		{
			pspace[i].x = person_pos[i].x + major_axis * cos(j / 57.3) * cos(person[i].ori[2]) - minor_axis * sin(j / 57.3) * sin(person[i].ori[2]);
			pspace[i].y = person_pos[i].y + major_axis * cos(j / 57.3) * sin(person[i].ori[2]) + minor_axis * sin(j / 57.3) * cos(person[i].ori[2]);
			cv::circle(image, person_pos[i], 6, cv::Scalar(255, 0, 0), 4, 8, 0); //lingkaran kecil
			cv::line(image, pspace[i], pspace[i], cv::Scalar(255, 0, 0), 1, 8, 0); //lingkaran besar
			cv::line(image, person_pos[i], person_ori[i], cv::Scalar(255, 0, 0), 4, 8, 0); //arah hadap robot
			sprintf(buf, "%d", i + 1);
			cv::putText(image, buf, cv::Point(person_pos[i].x + 2, person_pos[i].y + 2), 1, 2, cv::Scalar(255, 0, 0), 2, 8, true);
		}
	}
}

void CObstacle::interaction_space(int clientID, int n_obstacle)
{
	int i, j;
	char buf[5];
	/*
	// Retrieve data from the simulator - person 0 position and orientation
	simxGetObjectHandle(clientID, "obst0", &obstaclehandle0, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle0, -1, obstacle[0].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[0].pos[0],p[0].pos[1],p[0].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle0, -1, obstacle[0].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[0].ori[0],p[0].ori[1],p[0].ori[2]);

	// Retrieve data from the simulator - person 0 position and orientation
	simxGetObjectHandle(clientID, "obst1", &obstaclehandle1, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle1, -1, obstacle[1].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[0].pos[0],p[0].pos[1],p[0].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle1, -1, obstacle[1].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[0].ori[0],p[0].ori[1],p[0].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst2", &obstaclehandle2, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle2, -1, obstacle[2].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle2, -1, obstacle[2].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst3", &obstaclehandle3, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle3, -1, obstacle[3].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle3, -1, obstacle[3].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst4", &obstaclehandle4, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle4, -1, obstacle[4].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle4, -1, obstacle[4].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst5", &obstaclehandle5, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle5, -1, obstacle[5].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle5, -1, obstacle[5].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst6", &obstaclehandle6, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle6, -1, obstacle[6].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle6, -1, obstacle[6].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst7", &obstaclehandle7, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle7, -1, obstacle[7].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle7, -1, obstacle[7].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst8", &obstaclehandle8, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle8, -1, obstacle[8].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle8, -1, obstacle[8].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);

	// Retrieve data from the simulator - person 2 position and orientation
	simxGetObjectHandle(clientID, "obst9", &obstaclehandle9, simx_opmode_oneshot_wait);
	if (simxGetObjectPosition(clientID, obstaclehandle9, -1, obstacle[9].pos, simx_opmode_streaming) == simx_error_noerror)
	//printf("x = %.2f \t y = %.2f \t z = %.2f \n",p[1].pos[0],p[1].pos[1],p[1].pos[2]);
	if (simxGetObjectOrientation(clientID, obstaclehandle9, -1, obstacle[9].ori, simx_opmode_streaming) == simx_error_noerror)
	//printf("a = %.2f \t b = %.2f \t g = %.2f \n",p[1].ori[0],p[1].ori[1],p[1].ori[2]);
	*/

	for (i = 0; i<n_obstacle; i++)
	{
		// Calculate obstacle position from simulator to image
		obstacle_pos[i].x = 50 * obstacle[i].pos[0] + 250;
		obstacle_pos[i].y = 50 * obstacle[i].pos[1] + 250;
		// Calculate obstacle orientation from simulator to image
		obstacle_ori[i].x = obstacle_pos[i].x + 15 * cos(obstacle[i].ori[2]);
		obstacle_ori[i].y = obstacle_pos[i].y + 15 * sin(obstacle[i].ori[2]);
		// obstacle personal space
		for (j = -180; j<180; j++)
		{
			ospace[i].x = obstacle_pos[i].x + major_axis*cos(j / 57.3)*cos(obstacle[i].ori[2]) - minor_axis*sin(j / 57.3)*sin(obstacle[i].ori[2]);
			ospace[i].y = obstacle_pos[i].y + major_axis*cos(j / 57.3)*sin(obstacle[i].ori[2]) + minor_axis*sin(j / 57.3)*cos(obstacle[i].ori[2]);
			//cv::circle(image, obstacle_pos[i], 6, cv::Scalar(0, 255, 0), 4, 8, 0);
			//cv::line(image, ospace[i], ospace[i], cv::Scalar(0, 255, 0), 1, 8, 0);
			//cv::line(image, obstacle_pos[i], obstacle_ori[i], cv::Scalar(0, 255, 0), 1, 8, 0);
			sprintf(buf, "%d", i + 1);
			//cv::putText(image, buf, cv::Point(obstacle_pos[i].x + 2, obstacle_pos[i].y + 2), 1, 2, cv::Scalar(0, 255, 0), 2, 8, true);
		}
	}
}

void CRobot::interaction_space(int clientID)
{
	int i;
	// Retrieve data from the simulator - robot position and orientation
	//simxGetObjectHandle(clientID,"Pioneer_p3dx",&robothandle,simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "robotino", &robothandle, simx_opmode_oneshot_wait);

	if (simxGetObjectPosition(clientID, robothandle, -1, robot.pos, simx_opmode_streaming) == simx_error_noerror)
		//printf("x = %.2f \t y = %.2f \t z = %.2f\n",robot.pos[0],robot.pos[1],robot.pos[2]);
	if (simxGetObjectOrientation(clientID, robothandle, -1, robot.ori, simx_opmode_streaming) == simx_error_noerror)
		//printf("a = %.2f \t b = %.2f \t g = %.2f\n",robot.ori[0],robot.ori[1],robot.ori[2]);

		// Calculate robot position from simulator to image
		robot_pos.x = 50 * robot.pos[0] + 250;// +450;
	robot_pos.y = 50 * robot.pos[1] + 250;// +300;

	// Calculate robot orientation from simulator to image
	robot_ori.x = robot_pos.x + 15 * cos(robot.ori[2]);
	robot_ori.y = robot_pos.y + 15 * sin(robot.ori[2]);

	// Robot personal space
	for (i = -180; i < 180; i++)
	{
		pspace.x = robot_pos.x + major_axis * cos(i / 57.3) * cos(robot.ori[2]) - minor_axis * sin(i / 57.3) * sin(robot.ori[2]);
		pspace.y = robot_pos.y + major_axis * cos(i / 57.3) * sin(robot.ori[2]) + minor_axis * sin(i / 57.3) * cos(robot.ori[2]);
		cv::circle(image, robot_pos, 6, cv::Scalar(255, 255, 0), 4, 8, 0); //lingkaran kecil
		cv::line(image, pspace, pspace, cv::Scalar(255, 255, 0), 1, 8, 0); //lingkaran besar
		cv::line(image, robot_pos, robot_ori, cv::Scalar(255, 255, 0), 4, 8, 0); //arah depan robot
	}
}

void CGoal::interaction_space(int clientID)
{
	//char buf[10];
	// Retrieve data from the simulator - goal position
	simxGetObjectHandle(clientID, "Goal", &goalhandle, simx_opmode_oneshot_wait);

	if (simxGetObjectPosition(clientID, goalhandle, -1, goal.pos, simx_opmode_streaming) == simx_error_noerror)
		//printf("x = %.2f \t y = %.2f \t z = %.2f \n",goal.pos[0],goal.pos[1],goal.pos[2]);
		// Calculate goal position from simulator to image

		goal_pos.x = 50 * goal.pos[0] + 250;// +450;
	goal_pos.y = 50 * goal.pos[1] + 250;// +300;
	cv::circle(image, goal_pos, 4, cv::Scalar(255, 0, 255), 4, 8, 0);
	cv::putText(image, "Goal", cv::Point(goal_pos.x + 2, goal_pos.y + 2), 1, 2, cv::Scalar(255, 0, 255), 2, 8, true);
}

void CRobot::laser(int clientID)
{
	int i;
	//FILE *save;
	// Retrieve data from the simulator - laser range distance
	simxGetStringSignal(clientID, "measuredDataAtThisTime", &laserScannerData1, &dataSize, simx_opmode_streaming);
	//simxGetStringSignal(clientID, "ScannerData2", &laserScannerData2, &dataSize, simx_opmode_streaming);
	if (simxGetStringSignal(clientID, "measuredDataAtThisTime", &laserScannerData1, &dataSize, simx_opmode_buffer) == simx_error_noerror)
	{
		//save=fopen("angle.txt","w");
		for (i = 0; i < dataSize / (4 * 3); i++)
		{
			lrf[0].x = ((simxFloat*)(laserScannerData1 + 4 * 3 * i))[0];
			lrf[0].y = ((simxFloat*)(laserScannerData1 + 4 * 3 * i))[1];
			lrf[0].z = ((simxFloat*)(laserScannerData1 + 4 * 3 * i))[2];
			lrf[0].urg.x = (50 * robot.pos[0] + 250) + (lrf[0].x * 50 * cos(robot.ori[2]) - lrf[0].y * 50 * sin(robot.ori[2]));
			lrf[0].urg.y = (50 * robot.pos[1] + 250) + (lrf[0].x * 50 * sin(robot.ori[2]) + lrf[0].y * 50 * cos(robot.ori[2]));
			//lrf[0].dist[i]=sqrt(pow(lrf[0].urg.x-(50 * (robot.pos[0] + 0.25) + 625), 2) + pow(lrf[0].urg.y - (50 * robot.pos[1] + 250), 2));

			//simxAddStatusbarMessage(clientID, laserScannerData1, simx_opmode_oneshot);

			// Projecting the laser data to robot center
			lrf[0].dist_from_robot_center = sqrt(pow((float)lrf[0].urg.x - robot_pos.x, 2) + pow((float)lrf[0].urg.y - robot_pos.y, 2));
			lrf[0].angle = atan2((float)lrf[0].urg.y - robot_pos.y, (float)lrf[0].urg.x - robot_pos.x);
			lrf[0].from_robot_center[i].x = robot_pos.x + (lrf[0].dist_from_robot_center * cos(lrf[0].angle));
			lrf[0].from_robot_center[i].y = robot_pos.y + (lrf[0].dist_from_robot_center * sin(lrf[0].angle));
			lrf[0].dist[i] = lrf[0].dist_from_robot_center * 2;
			//fprintf(save,"%d. %.2f \t %.2f \t %.2f \n",i,57.3*(i-342) * 0.00612, (float)lrf[2].dist[i], (float)lrf[2].dist[i] / 50);
			cv::circle(image, lrf[0].from_robot_center[i], 1, cv::Scalar(0, 0, 255), 1, 8, 0);
		}
		//fclose(save);
	}

	simxGetStringSignal(clientID, "measuredDataAtThisTime2", &laserScannerData2, &dataSize, simx_opmode_streaming);
	//simxGetStringSignal(clientID, "ScannerData2", &laserScannerData2, &dataSize, simx_opmode_streaming);
	if (simxGetStringSignal(clientID, "measuredDataAtThisTime2", &laserScannerData2, &dataSize, simx_opmode_buffer) == simx_error_noerror)
	{
		//save=fopen("angle.txt","w");
		for (i = 0; i < dataSize / (4 * 3); i++)
		{
			lrf[1].x = ((simxFloat*)(laserScannerData2 + 4 * 3 * i))[0];
			lrf[1].y = ((simxFloat*)(laserScannerData2 + 4 * 3 * i))[1];
			lrf[1].z = ((simxFloat*)(laserScannerData2 + 4 * 3 * i))[2];
			lrf[1].urg.x = (50 * robot.pos[0] + 250) + (lrf[1].x * 50 * cos(robot.ori[2]) - lrf[1].y * 50 * sin(robot.ori[2]));
			lrf[1].urg.y = (50 * robot.pos[1] + 250) + (lrf[1].x * 50 * sin(robot.ori[2]) + lrf[1].y * 50 * cos(robot.ori[2]));
			lrf[1].urg.x = ((float)lrf[1].urg.x - robot_pos.x)*cos(180 * phi / 180) - ((float)lrf[1].urg.y - robot_pos.y)*sin(180 * phi / 180) + robot_pos.x;
			lrf[1].urg.y = ((float)lrf[1].urg.x - robot_pos.x)*sin(180 * phi / 180) + ((float)lrf[1].urg.y - robot_pos.y)*cos(180 * phi / 180) + robot_pos.y;

			//lrf[0].dist[i]=sqrt(pow(lrf[0].urg.x-(50 * (robot.pos[0] + 0.25) + 625), 2) + pow(lrf[0].urg.y - (50 * robot.pos[1] + 250), 2));

			//simxAddStatusbarMessage(clientID, laserScannerData1, simx_opmode_oneshot);

			// Projecting the laser data to robot center
			lrf[1].dist_from_robot_center = sqrt(pow((float)lrf[1].urg.x - robot_pos.x, 2) + pow((float)lrf[1].urg.y - robot_pos.y, 2));
			lrf[1].angle = atan2((float)lrf[1].urg.y - robot_pos.y, (float)lrf[1].urg.x - robot_pos.x);
			lrf[1].from_robot_center[i].x = robot_pos.x + (lrf[1].dist_from_robot_center * cos(lrf[1].angle));
			lrf[1].from_robot_center[i].y = robot_pos.y + (lrf[1].dist_from_robot_center * sin(lrf[1].angle));
			lrf[1].dist[i] = lrf[1].dist_from_robot_center * 2;
			//fprintf(save,"%d. %.2f \t %.2f \t %.2f \n",i,57.3*(i-342) * 0.00612, (float)lrf[2].dist[i], (float)lrf[2].dist[i] / 50);
			//fprintf(savedata2, "%d %f\n", i, (float)lrf[1].dist[i] / 100);
			//fprintf(savedata2, "%d \t %f \t %f\n", i, 57.3*(i - 342) * 0.00612, ((float)lrf[1].angle * 180) / phi);
			cv::circle(image, lrf[1].from_robot_center[i], 1, cv::Scalar(0, 0, 255), 1, 8, 0);
		}
		//fclose(save);
	}
}

AttractiveForce CRobot::goal(int step, float Vt, cv::Point gpos, float Vd)
{
	float dx, dy, k;
	AttractiveForce by_goal;

	// Force towards a goal
	if (step == 0)
		by_goal.dist = 5000;
	else
		by_goal.dist = sqrt(pow((float)gpos.x - (float)robot_pos.x, 2) + pow((float)gpos.y - (float)robot_pos.y, 2));

	dx = (float)gpos.x - (float)robot_pos.x;
	dy = (float)gpos.y - (float)robot_pos.y;
	by_goal.angle = atan2((float)dy, (float)dx);
	k = 1 / tau;
	by_goal.force = fabs(m * k * (Vd - Vt));

	// Drawing the vector of force

	// Physical force
	by_goal.pos.x = robot_pos.x +/*by_goal.force*/50 * cos(by_goal.angle) / (scale/*m*/);
	by_goal.pos.y = robot_pos.y +/*by_goal.force*/50 * sin(by_goal.angle) / (scale/*m*/);
	cv::line(image, robot_pos, by_goal.pos, cv::Scalar(0, 0, 255), 2, 8, 0);
	return(by_goal);
}

RepulsiveForce CRobot::dynamics(int n_person, struct group prsn, float Ad, float Bd, float lambda)
{
	int i, j;
	float dx1, dy1, dx2, dy2, Fsp[20], Fpp[20], min_dist;
	cv::Point temp, fspr[20], fppr[20], temp1[20];
	CPerson p;
	RepulsiveForce by_dynamics;

	// Person's force to the robot
	temp.x = 0;
	temp.y = 0;
	min_dist = 8000;
	by_dynamics.force = 0;
	by_dynamics.angle = 0;

	for (i = 0; i < n_person; i++)
	{
		p.person[i].dist = sqrt(pow((float)robot_pos.x - prsn.person[i].pos.x, 2) + pow((float)robot_pos.y - prsn.person[i].pos.y, 2));
		if (min_dist > p.person[i].dist)
			min_dist = p.person[i].dist;

		dx1 = (float)robot_pos.x - (float)prsn.person[i].pos.x;
		dy1 = (float)robot_pos.y - (float)prsn.person[i].pos.y;
		p.person[i].alpha = atan2((float)dy1, (float)dx1);
		dx2 = (float)prsn.person[i].pos.x - (float)robot_pos.x;
		dy2 = (float)prsn.person[i].pos.y - (float)robot_pos.y;
		robot.alpha = atan2((float)dy2, (float)dx2);
		by_dynamics.rel_angle = robot.alpha - robot.ori[2];

		if (by_dynamics.rel_angle > 3.14)
			by_dynamics.rel_angle = by_dynamics.rel_angle - 3.14;
		else if (by_dynamics.rel_angle < -3.14)
			by_dynamics.rel_angle = by_dynamics.rel_angle + 3.14;
		else
			by_dynamics.rel_angle = by_dynamics.rel_angle;

		// Calculate the radii of person i
		p.person[i].radii.x = prsn.person[i].pos.x + major_axis * cos(p.person[i].alpha - prsn.person[i].ori) * cos(prsn.person[i].ori) - minor_axis * sin(p.person[i].alpha - prsn.person[i].ori) * sin(prsn.person[i].ori);
		p.person[i].radii.y = prsn.person[i].pos.y + major_axis * cos(p.person[i].alpha - prsn.person[i].ori) * sin(prsn.person[i].ori) + minor_axis * sin(p.person[i].alpha - prsn.person[i].ori) * cos(prsn.person[i].ori);
		p.person[i].radius = sqrt(pow((float)p.person[i].radii.x - (float)prsn.person[i].pos.x, 2) + pow((float)p.person[i].radii.y - (float)prsn.person[i].pos.y, 2));

		// Calculate the radii of the robot
		robot.radii.x = robot_pos.x + major_axis * cos(robot.alpha - robot.ori[2]) * cos(robot.ori[2]) - minor_axis * sin(robot.alpha - robot.ori[2]) * sin(robot.ori[2]);
		robot.radii.y = robot_pos.y + major_axis * cos(robot.alpha - robot.ori[2]) * sin(robot.ori[2]) + minor_axis * sin(robot.alpha - robot.ori[2]) * cos(robot.ori[2]);
		robot.radius = sqrt(pow((float)robot.radii.x - (float)robot_pos.x, 2) + pow((float)robot.radii.y - (float)robot_pos.y, 2));

		if (p.person[i].dist > (robot.radius + p.person[i].radius))// || abs(robot.alpharobot.ori[2])>1.57)
		{
			by_dynamics.k = 0;
			Fsp[i] = 0;
			Fpp[i] = 0;
		}
		else
		{
			if (robot.alpha - prsn.person[i].ori</*0.5*/phi || robot.alpha - prsn.person[i].ori>/*-0.5*/-phi)
				cv::circle(image, p.person[i].radii, 2, cv::Scalar(255, 0, 0), 2, 8, 0);

			if (robot.ori[2] - p.person[i].alpha</*0.5*/phi || robot.ori[2] - p.person[i].alpha>/*-0.5*/-phi)
				cv::circle(image, robot.radii, 2, cv::Scalar(0, 0, 255), 2, 8, 0);

			Ad=fis((float)p.person[i].dist,by_dynamics.rel_angle);
			by_dynamics.k = Ad;
			Fsp[i] = Ad * exp(((robot.radius + p.person[i].radius) - p.person[i].dist) / Bd) * (lambda + (1 - lambda) * ((1 + cos(robot.alpha)) / 2));
			Fpp[i] = Ad * (0.01 * (robot.radius + p.person[i].radius) - 0.01 * p.person[i].dist);
		}

		// Total vector
		temp.x += Fsp[i] * cos(p.person[i].alpha) + Fpp[i] * cos(p.person[i].alpha);
		temp.y += Fsp[i] * sin(p.person[i].alpha) + Fpp[i] * sin(p.person[i].alpha);
	}

	// Total force caused by the presence of people
	by_dynamics.force = sqrt(pow((float)temp.x, 2) + pow((float)temp.y, 2));
	by_dynamics.angle = atan2((float)temp.y, (float)temp.x);

	// Drawing the resultant vector of all dynamic's forces
	if (fabs(by_dynamics.force) > 0)
	{
		by_dynamics.pos.x = robot_pos.x +/*by_dynamics.force*/50 * cos(by_dynamics.angle) / (scale/*m*/);
		by_dynamics.pos.y = robot_pos.y +/*by_dynamics.force*/50 * sin(by_dynamics.angle) / (scale/*m*/);
		cv::line(image, robot_pos, by_dynamics.pos, cv::Scalar(255, 0, 0), 2, 8, 0);
	}
	by_dynamics.dist = min_dist; // minimum distance by moving object
	by_dynamics.force1 = sqrt(pow((float)Fsp[0] * cos(p.person[0].alpha) + Fpp[0] * cos(p.person[0].alpha), 2) + pow((float)Fsp[0] * sin(p.person[0].alpha) + Fpp[0] * sin(p.person[0].alpha), 2));
	by_dynamics.force2 = sqrt(pow((float)Fsp[1] * cos(p.person[1].alpha) + Fpp[1] * cos(p.person[1].alpha), 2) + pow((float)Fsp[1] * sin(p.person[1].alpha) + Fpp[1] * sin(p.person[1].alpha), 2));
	by_dynamics.dist1 = p.person[0].dist;
	by_dynamics.dist2 = p.person[1].dist;
	return(by_dynamics);
}

RepulsiveForce CRobot::statics(float As, float Bs, float lambda)
{
	int i;
	cv::Point temp1, fswr[10], fpwr[10];
	float Fsw[700], Fpw[700], angle, min_dist1, min_dist2, min_dist3, min_angle1, min_angle2, min_angle3;
	RepulsiveForce by_statics;

	// Object's force to the robot - wall, furniture, etc

	// right side
	min_dist1 = 500;
	for (i = 86; i < 257; i++)
	{
		if (min_dist1 > lrf[0].dist[i])
		{
			min_dist1 = lrf[0].dist[i];
			min_angle1 = (i - 342) * 0.00612;
		}
	}

	// left side
	min_dist2 = 500;
	for (i = 428; i < 684 - 86; i++)
	{
		if (min_dist2 > lrf[0].dist[i])
		{
			min_dist2 = lrf[0].dist[i];
			min_angle2 = (i - 342) * 0.00612;
		}
	}

	if (min_dist1 < min_dist2)
	{
		lrf[0].min_dist1 = min_dist1;
		lrf[0].beta1 = min_angle1;
	}
	else
	{
		lrf[0].min_dist1 = min_dist2;
		lrf[0].beta1 = min_angle2;
	}

	// front side
	min_dist3 = 500;
	for (i = 257; i < 427; i++)
	{
		if (min_dist3 > lrf[0].dist[i])
		{
			min_dist3 = lrf[0].dist[i];
			min_angle3 = (i - 342) * 0.00612;
		}
	}
	lrf[0].min_dist2 = min_dist3;
	lrf[0].beta2 = min_angle3;
	temp1.x = 0;
	temp1.y = 0;

	/*
	// side back
	min_dist1 = 500;
	for (i = 86; i < 257; i++)
	{
		if (min_dist1 > lrf[1].dist[i])
		{
			min_dist1 = lrf[1].dist[i];
			min_angle1 = (i - 342) * 0.00612;
		}
	}

	// side back
	min_dist2 = 500;
	for (i = 428; i < 684 - 86; i++)
	{
		if (min_dist2 > lrf[1].dist[i])
		{
			min_dist2 = lrf[1].dist[i];
			min_angle2 = (i - 342) * 0.00612;
		}
	}

	if (min_dist1 < min_dist2)
	{
		lrf[0].min_dist1 = min_dist1;
		lrf[0].beta1 = min_angle1;
	}
	else
	{
		lrf[0].min_dist1 = min_dist2;
		lrf[0].beta1 = min_angle2;
	}

	// back
	min_dist3 = 500;
	for (i = 257; i < 427; i++)
	{
		if (min_dist3 > lrf[1].dist[i])
		{
			min_dist3 = lrf[1].dist[i];
			min_angle3 = (i - 342) * 0.00612;
		}
	}
	lrf[1].min_dist2 = min_dist3;
	lrf[1].beta2 = min_angle3;
	*/

	for (i = 86; i < 684 - 86; i++)
	{
		lrf[0].alpha[i] = atan2((float)robot_pos.y - lrf[0].from_robot_center[i].y, (float)robot_pos.x - lrf[0].from_robot_center[i].x);
		robot.alpha = atan2((float)lrf[0].from_robot_center[i].y - robot_pos.y, (float)lrf[0].from_robot_center[i].x - robot_pos.x);

		// Total vector
		robot.radii.x = robot_pos.x + major_axis * cos(robot.alpha - robot.ori[2]) * cos(robot.ori[2]) - minor_axis * sin(robot.alpha - robot.ori[2]) * sin(robot.ori[2]);
		robot.radii.y = robot_pos.y + major_axis * cos(robot.alpha - robot.ori[2]) * sin(robot.ori[2]) + minor_axis * sin(robot.alpha - robot.ori[2]) * cos(robot.ori[2]);
		robot.radius = sqrt(pow((float)robot.radii.x - robot_pos.x, 2) + pow((float)robot.radii.y - robot_pos.y, 2));
		if (lrf[0].dist[i] > robot.radius + 25 || abs(robot.alpha - robot.ori[2]) > 1.57)
		{
			Fsw[i] = 0;
			Fpw[i] = 0;
		}
		else
		{
			//cv::circle(image,robot.radii,2,cv::Scalar(0,255,0),2,8,0);
			As = fis(lrf[0].dist[i] / 100, lrf[0].angle) + fis(lrf[1].dist[i] / 100, lrf[1].angle);
			Fsw[i] = As * exp((robot.radius - lrf[0].dist[i]) / Bs) * (lambda + (1 - lambda) * ((1 + cos(robot.alpha)) / 2));
			Fpw[i] = As * (robot.radius - lrf[0].dist[i]);
		}
		temp1.x += Fsw[i] * cos(lrf[0].alpha[i]) + Fpw[i] * cos(lrf[0].alpha[i]);
		temp1.y += Fsw[i] * sin(lrf[0].alpha[i]) + Fpw[i] * sin(lrf[0].alpha[i]);
	}

	// Drawing the vector of force

	// Social force
	fswr[i].x = robot_pos.x + Fsw[i] * cos(robot.alpha) / (scale * m);
	fswr[i].y = robot_pos.y + Fsw[i] * sin(robot.alpha) / (scale * m);

	//cv::line(image, robot_pos, fswr[i], cv::Scalar(0, 255, 0), 2, 8, 0);

	// Physical force
	fpwr[i].x = robot_pos.x + Fpw[i] * cos(robot.alpha) / (scale * m);
	fpwr[i].y = robot_pos.y + Fpw[i] * sin(robot.alpha) / (scale * m);

	//cv::line(image, robot_pos, fpwr[i], cv::Scalar(0, 255, 0), 2, 8, 0);

	// Total force caused by the presence of obstacle(s)
	by_statics.force = sqrt(pow((float)temp1.x, 2) + pow((float)temp1.y, 2));
	by_statics.angle = atan2((float)temp1.y, (float)temp1.x);

	// Drawing the resultant vector of all static's forces
	if (fabs(by_statics.force) > 0)
	{
		//by_dynamics.pos.x = robot_pos.x +/*by_dynamics.force*/50 * cos(by_dynamics.angle) / (scale/*m*/);
		//by_dynamics.pos.y = robot_pos.y +/*by_dynamics.force*/50 * sin(by_dynamics.angle) / (scale/*m*/);
		//cv::line(image, robot_pos, by_dynamics.pos, cv::Scalar(255, 0, 0), 2, 8, 0);

		by_statics.pos.x = robot_pos.x + /*by_statics.force*/50 * cos(by_statics.angle) / (scale/* m*/);
		by_statics.pos.y = robot_pos.y + /*by_statics.force*/50 * sin(by_statics.angle) / (scale/* m*/);
		cv::line(image, robot_pos, by_statics.pos, cv::Scalar(255, 0, 0), 2, 8, 0);
	}
	by_statics.dist = 0;
	return(by_statics);
}

float triangle(float a, float b, float c, float x)
{
	float d;
	if (x >= a && x < b)
		d = (x - a) / (b - a);
	else if (x >= b && x < c)
		d = (c - x) / (c - b);
	else
		d = 0;
	return(d);
}

float fis(float a, float b)
{
	float dist[5], angle[5], rb[25], fuzz, temp, temp1;
	int i, j;
	float VH, H, M, L, VL;
	VH = 500;
	H = 100;
	M = 50;
	L = 10;
	VL = 5;
	float r[16] = { VH, H, H, H, H, H, M, L, M, M, L, L, L, L, L, VL };

	// Fuzzification : Degree of MF of distance
	dist[0] = triangle(0, 0, 0.45, a);
	dist[1] = triangle(0, 0.45, 1.2, a);
	dist[2] = triangle(0.45, 1.2, 3.6, a);
	dist[3] = triangle(1.2, 3.6, 30, a);

	// Fuzzyfikasi : Degree of MF of angle
	angle[0] = triangle(0, 0, 60, fabs(b * 57.3));
	angle[1] = triangle(0, 60, 120, fabs(b * 57.3));
	angle[2] = triangle(60, 120, 180, fabs(b * 57.3));
	angle[3] = triangle(120, 180, 180, fabs(b * 57.3));

	// Rule Base
	temp = 0;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			rb[i * 4 + j] = dist[i] * angle[j];
			temp += rb[i * 4 + j];
		}
	}

	// Output (Defuzzyfikasi)
	temp1 = 0;
	for (i = 0; i < 4; i++)
	for (j = 0; j < 4; j++)
		temp1 += rb[i * 4 + j] * r[i * 4 + j];
	fuzz = temp1 / temp;
	return(fuzz);
}

void Scene()
{
	int i, j;
	cv::Vec3b color;
	cv::Point p1, p2;

	for (i = 0; i<500; i++)
	{
		color.val[0] = 255;
		color.val[1] = 255;
		color.val[2] = 255;

		for (j = 0; j<500; j++)
			image.at<cv::Vec3b>(i, j) = color;
	}

	// Draw vertical grid
	for (i = 0; i<20; i++)
	{
		p1.x = i * 25;
		p1.y = 0;
		p2.x = i * 25;
		p2.y = 500;
		cv::line(image, p1, p2, cv::Scalar(200, 200, 200), 1, 8, 0);
	}

	// Draw horizontal grid
	for (i = 0; i<20; i++)
	{
		p1.x = 0;
		p1.y = i * 25;
		p2.x = 500;
		p2.y = i * 25;
		cv::line(image, p1, p2, cv::Scalar(200, 200, 200), 1, 8, 0);
	}
}