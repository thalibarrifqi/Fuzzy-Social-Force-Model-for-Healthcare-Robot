#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <Windows.h>
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

extern "C"
{
#include "extApi.h"
#include "extApiPlatform.h"
}

struct lines
{
	cv::Point p;
};

struct pid_controller
{
	float kp, ki, kd;
};

struct obstacle
{
	struct lines pos[5000];
};

#include "HUM_FUZZY_SFM2.h"

using namespace std;

cv::Point fnav, gpos, rpos;
int Kk_gain, Kh_gain, effect_area;
float velx, velx_, vely, vely_, vtheta, min_dist1, min_dist2, max_speed, dalpha_goal_, gain;

void main()
{
	float Vt, rx, ry, dx, dy, Ftot_1, Ftot, alpha, err;
	float  dalpha_goal, Vt_, Fg_dist, acc_dalpha_goal, dif_dalpha_goal;
	double time = 0;
	double ts[100];
	int portComm = 19997;
	int clientID = 0;
	char bufferName[50], buf2[20];
	float simTime;

	int i, j, step, stat, episode, n_episode = 1;
	int api = 0;

	struct AttractiveForce Fg;
	struct RepulsiveForce Fp, Fw;
	struct lines position[5000];
	struct pid_controller pid;
	struct group p;
	struct obstacle obs[3];

	CRobot rbt;
	CGoal goal;
	CObstacle obst;
	CPerson prsn;

	clientID = simxStart((simxChar*)"127.0.0.1", portComm, true, true, 2000, 5);

	Vt = 0;
	Ftot_1 = 0;
	rpos.x = 0;
	rpos.y = 0;
	Fp.angle = 0;
	Fp.dist = 0;
	Fw.angle = 0;
	Fw.dist = 0;
	Fg_dist = 0;
	vtheta = 0;
	err = 0;
	dalpha_goal = 0;
	acc_dalpha_goal = 0;
	dif_dalpha_goal = 0;
	velx_ = 0;
	vely_ = 0;

	effect_area = 50;
	Kk_gain = 10;
	Kh_gain = 10;
	max_speed = 1.0;
	min_dist1 = 100;
	min_dist2 = 100;
	step = 0;

	// Robot and obstacle position from V-Rep
	position[-1].p.x = rbt.robot_pos.x;
	position[-1].p.y = rbt.robot_pos.y;


	/*

	for (i = 0; i < 0; i++)
	{
	obs[i].pos[-1].p.x = obst.obstacle_pos[i].x;
	obs[i].pos[-1].p.y = obst.obstacle_pos[i].y;
	}

	obs[1].pos[-1].p.x = obst.obstacle_pos[1].x;
	obs[1].pos[-1].p.y = obst.obstacle_pos[1].y;
	obs[2].pos[-1].p.x = obst.obstacle_pos[2].x;
	obs[2].pos[-1].p.y = obst.obstacle_pos[2].y;
	obs[3].pos[-1].p.x = obst.obstacle_pos[3].x;
	obs[3].pos[-1].p.y = obst.obstacle_pos[3].y;

	*/
	/*

	obs[4].pos[-1].p.x = prsn.person_pos[0].x;
	obs[4].pos[-1].p.y = prsn.person_pos[0].y;
	obs[5].pos[-1].p.x = prsn.person_pos[1].x;
	obs[5].pos[-1].p.y = prsn.person_pos[1].y;

	*/
	
	while (1)
	{
		Scene();
		api++;
		simxAddStatusbarMessage(clientID, "API CONNECTED", simx_opmode_oneshot);
		simxStartSimulation(clientID, simx_opmode_oneshot);
		simxGetFloatSignal(clientID, "mySimulationTime", &simTime, simx_opmode_streaming);

		if (clientID != -1)
		{
			double t = (double)cv::getTickCount();

			rbt.interaction_space(clientID);
			goal.interaction_space(clientID);
			rbt.laser(clientID);
			//prsn.interaction_space(clientID, 3);
			//obst.interaction_space(clientID, 10);

			gpos.x = goal.goal_pos.x; //GOAL POSITION
			gpos.y = goal.goal_pos.y;

			rpos.x = rbt.robot_pos.x; //ROBOT POSITION
			rpos.y = rbt.robot_pos.y;

			for (i = 0; i < 3; i++)
			{
				//p.obstacle[i].pos.x = obst.obstacle_pos[i].x;
				//p.obstacle[i].pos.y = obst.obstacle_pos[i].y;

				p.person[i].pos.x = prsn.person_pos[i].x;
				p.person[i].pos.y = prsn.person_pos[i].y;

				p.person[i].ori = prsn.person[i].ori[2];
				//p.obstacle[i].ori = obst.obstacle[i].ori[3];

				//obs[i].pos[step].p.x = p.obstacle[i].pos.x;
				//obs[i].pos[step].p.y = p.obstacle[i].pos.y;

				obs[i].pos[step].p.x = p.person[i].pos.x;
				obs[i].pos[step].p.y = p.person[i].pos.y;
			}

			//MAIN CODE
			//printf("Posisi Goal X : Y = %d : %d\n",gpos.x, gpos.y);
			//printf("Posisi Robot X : Y = %d : %d\n", rpos.x, rpos.y);
			//printf("Posisi Obstacle X : Y = %d : %d\n", p.obstacle[0].pos.x, p.obstacle[0].pos.y);
			//printf("Data laser x, y, z = %f, %f, %f\n", lrf[0].x, lrf[0].y, lrf[0].z);
			//printf("Object to Robot dist = %f\n", lrf[2].dist[i]);
			//printf("Object to Robot angle = %f\n", lrf[0].angle);

			Fg = rbt.goal(step, Vt, gpos, 1);
			Fp = rbt.dynamics(3, p, Kk_gain, effect_area, 1);
			Fw = rbt.statics(Kh_gain, effect_area, 1);
			//printf("Fg\t Fp\t Fw\t\n%.2f\t %.2f\t %.2f\t\n", Fg.force, Fp.force, Fw.force);
			rx = Fg.force*cos(Fg.angle) + Fp.force*cos(Fp.angle) + Fw.force*cos(Fw.angle); 	//rx = Fg.force * cos(Fg.angle) + Fw.force * cos(Fw.angle);
			ry = Fg.force*sin(Fg.angle) + Fp.force*sin(Fp.angle) + Fw.force*sin(Fw.angle);	//ry = Fg.force * sin(Fg.angle) + Fw.force * sin(Fw.angle);

			Ftot = sqrt(pow((float)rx, 2) + pow((float)ry, 2));

			Ftot = 0.5 * (Ftot + Ftot_1);

			dx = rx - rbt.robot_pos.x;
			dy = ry - rbt.robot_pos.y;

			alpha = atan2(ry, rx); //sudut fnav

			if (alpha < -1.57) //supaya ga mental kebelakang
				alpha = 1.1;
			if (alpha > 1.57)
				alpha = -1.1;

			// Gambar Navigasi
			fnav.x = rbt.robot_pos.x + /*Ftot */ 50 * cos(alpha) / (scale/*m*/);
			fnav.y = rbt.robot_pos.y + /*Ftot */ 50 * sin(alpha) / (scale/*m*/);
			cv::line(image, rbt.robot_pos, fnav, cv::Scalar(255, 0, 0), 2, 8, 0);

			Vt = (Ftot * 0.1 / m);

			if (Fw.force == 0) Vt = max_speed;
			if (Fp.force == 0) Vt = max_speed;
			if (Vt > max_speed)	Vt = max_speed;

			position[step].p.x = rpos.x;
			position[step].p.y = rpos.y;

			dalpha_goal = Fg.angle - rbt.robot.ori[2];
			if (dalpha_goal > phi)
				dalpha_goal -= 2 * phi;
			if (dalpha_goal < -phi)
				dalpha_goal += 2 * phi;
			acc_dalpha_goal += dalpha_goal;
			dif_dalpha_goal = dalpha_goal - dalpha_goal_;

			// ROBOT CONTROL
			rbt.robot.velx = Vt * cos((float)alpha);
			rbt.robot.vely = Vt * sin((float)alpha);
			
			pid.kp = 4;
			pid.ki = 0.0005;
			pid.kd = 24;

			//pid.kp = 8;
			//pid.ki = 0.0005;
			//pid.kd = 20;

			//pid.kp = 25;
			//pid.ki = 0.15;
			//pid.kd = 30;

			//pid.kp = 10;
			//pid.ki = 0.005;
			//pid.kd = 20;

			vtheta = pid.kp * dalpha_goal + pid.ki * acc_dalpha_goal + pid.kd * dif_dalpha_goal;

			//printf("%d \t %.1f \t %f\n", api, simTime, vtheta); //Mengamati vtheta

			Ftot_1 = Ftot;
			Vt_ = Vt;
			dalpha_goal_ = dalpha_goal;

			Fg_dist = Fg.dist;
			velx_ = rbt.robot.velx;
			vely_ = rbt.robot.vely;

			simxSetFloatSignal(clientID, "velx", velx_, simx_opmode_oneshot); //velocity X
			simxSetFloatSignal(clientID, "vely", vely_, simx_opmode_oneshot);  //velocity Y
			simxSetFloatSignal(clientID, "vtheta", vtheta, simx_opmode_oneshot); //velocity of rotation

			gain = fis(lrf[2].dist[i] / 100, lrf[0].angle);

			t = (double)cv::getTickCount();
			time = t / ((double)cv::getTickFrequency() * 1000);
			ts[step] = time;

			//printf("%d \t %.1f \t %.2f \t %.2f \t %.2f \t %.2f \t %f\n", api, simTime, lrf[2].dist[i] / 100, lrf[0].angle, gain, Fg.angle, vtheta);

			for (i = 4; i < step; i++)
			{
				cv::line(image, position[i - 1].p, position[i].p, cv::Scalar(0, 255, 255), 2, 8, 0);
			}

			cv::flip(image, image, 0);
			//sprintf(buf2, "%d", api);
			//cv::putText(image, buf2, cv::Point(10, 50), 1, 2, cv::Scalar(0, 0, 0), 3, 6, false);
			//sprintf(bufferName, "image_scene_3\\Data-%d.jpg", api);
			//imwrite(bufferName, image);
			
			cv::imshow("Simulation", image);

			if (Fg.dist < 25)	// reach the goal 
			{
				printf("Time to reach the goal = %f\n", simTime);
				//sprintf(buf3, "scheme\\\\%d.jpg", episode + 1);
				simxStopSimulation(clientID, simx_opmode_oneshot);
				Sleep(30000);
			}

			position[step - 1].p.x = rpos.x;
			position[step - 1].p.y = rpos.y;

			if (min_dist1 > Fw.dist1 / 100)
				min_dist1 = Fw.dist1 / 100;

			if (min_dist2 > Fw.dist2 / 100)
				min_dist2 = Fw.dist2 / 100;

			cv::waitKey(1);
		}
		step++;
		if (step > 1000)
			step = 0;
		//cv::waitKey(1);
		
	}//while
	
}