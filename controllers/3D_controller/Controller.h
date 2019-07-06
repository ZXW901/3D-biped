#ifndef CONTROLLER_H_
#define CONTROLLER_H_

//#include <stdlib.h>
//#include <iostream>
//#include <new>
//#include "Classes.h"
#include "Cleg.h"
#include "Config.h"
#include "CRobot.h"
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/LU> 


using namespace std;

class Controller
{
public:
	Controller();
	~Controller();

	BodyVelocity CalBodyVel(double, double);
	Angle CalcStlegAngle(double, double, double);
	int sign(double);
	void Log(double, double);
	void Log(double, double, double, double);
	void run();
	void GetKeyBoard(int);
	CRobot crobot;
	Cleg leg[2];
	BodyVelocity BodyVel;
	CubicValue	l_sw, d_sw, hip_arg;	//摆动腿收缩，摆动腿摆动

	Cartesian hipOffset;
	Angle Target_angle[2];
	Cartesian Cartesian_foot[2];
	XYZ xyz;
	Force gravity = Force(0, 0, (M_TORSO+ 2*M_LEG) *G);
	Torque gravitycom;		//gravity compensation

	ofstream oFile;
	//Torque L_u, R_u;

	double dt;	//basicTimeStep
	double u[6];	//output torque
	double T,t;		//run time;since last stance leg switch
	int	state,stanceLeg;	//controller state;current stance leg
	double	x_st_e, x_sw_e;		//Persistent variable to keep track of states at last switch
	double	x_est, y_est;		//Persistent variable to keep track of estimated position
	double	dx_est, dy_est, dx_est_e, dy_est_e;	//Persistent variable to keep track of estimated velocity
	double dx_est_real, dy_est_real;
	double dx_tgt, dy_tgt;		//Persistent variable to keep track of target velocity
	double dx_cmd, dy_cmd;
	double dTarget_angle[6];
	double Target_angle_e[6];
	double stanceAngle;
	int key;
};

#endif // !CONTROLLER_H_
