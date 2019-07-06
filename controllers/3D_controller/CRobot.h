#ifndef CROBOT_H_
#define CROBOT_H_

#include <stdlib.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Joystick.hpp>
#include "Classes.h"

using namespace webots;

class CRobot
{
public:
	CRobot();
	~CRobot();
	void UpdateAnalogAll();

	Robot *robot;
	int timeStep; 

	Motor *L_motor[3], *R_motor[3];//, *Z_motor;
	PositionSensor *L_motor_sensor[3], *R_motor_sensor[3];
	TouchSensor *L_foot, *R_foot;
	InertialUnit *Imu;
	GPS *Gps;
	Keyboard KeyBoard;
	Joystick JoyStick;

	bool StarTime;
	double X_sensor[6];
	double X_sensor_e[6];
	double dX_sensor[6];
	double F_touch[6];
	double Imu_value[3];		// pitch roll yaw
	double dImu_value[3];
	double Imu_value_e[3];
	double Gps_value[3];
	double dGps_value[3];
	double Gps_value_e[3];		//double Imu_roll, Imu_pitch, Imu_yaw;
};

#endif // !CROBOT_H_

