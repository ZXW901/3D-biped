#include "CRobot.h"

using namespace std;
using namespace webots;

CRobot::CRobot()
{
	robot = new Robot();
	timeStep = (int)robot->getBasicTimeStep();
	StarTime = true;
	X_sensor[6] = { 0 };
	X_sensor_e[6] = { 0 };
	dX_sensor[6] = { 0 };
	F_touch[6] = { 0 };

	Imu_value[3] = { 0 };
	dImu_value[3] = { 0 };
	Imu_value_e[3] = { 0 };

	const char *R_MOTOR_NAME[3] = { "R_roll_m","R_thigh_m","R_knee_m" };
	const char *L_MOTOR_NAME[3] = { "L_roll_m","L_thigh_m","L_knee_m" };
	const char *R_MOTOR_SENSOR_NAME[3] = { "X_r_roll","X_r_thigh","X_r_knee" };
	const char *L_MOTOR_SENSOR_NAME[3] = { "X_l_roll","X_l_thigh","X_l_knee" };

	for (int i = 0; i < 3; i++)
	{
		R_motor[i] = robot->getMotor(R_MOTOR_NAME[i]);
		L_motor[i] = robot->getMotor(L_MOTOR_NAME[i]);
		R_motor_sensor[i] = robot->getPositionSensor(R_MOTOR_SENSOR_NAME[i]);
		L_motor_sensor[i] = robot->getPositionSensor(L_MOTOR_SENSOR_NAME[i]);


		R_motor[i]->enableTorqueFeedback(timeStep);
		L_motor[i]->enableTorqueFeedback(timeStep);
		R_motor_sensor[i]->enable(timeStep);
		L_motor_sensor[i]->enable(timeStep);

	}
		R_foot = robot->getTouchSensor("R_foot");
		L_foot = robot->getTouchSensor("L_foot");
		Imu = robot->getInertialUnit("IMU");
		Gps = robot->getGPS("GPS");
		R_foot->enable(timeStep);
		L_foot->enable(timeStep);
		Imu->enable(timeStep);
		Gps->enable(timeStep);
		KeyBoard.enable(timeStep);
		JoyStick.enable(timeStep);
}

CRobot::~CRobot()
{

}

void CRobot::UpdateAnalogAll()
{
	const double *lf_touch_value = L_foot->getValues();
	const double *rf_touch_value = R_foot->getValues();
	const double *vg_value = Imu->getRollPitchYaw();
	const double *gps_value = Gps->getValues();
	
	F_touch[0] = sqrt(pow(lf_touch_value[0], 2) + pow(lf_touch_value[1], 2) + pow(lf_touch_value[2], 2));
	F_touch[1] = sqrt(pow(rf_touch_value[0], 2) + pow(rf_touch_value[1], 2) + pow(rf_touch_value[2], 2));

	X_sensor[0] = R_motor_sensor[0]->getValue();
	X_sensor[1] = R_motor_sensor[1]->getValue();
	X_sensor[2] = R_motor_sensor[2]->getValue();
	X_sensor[3] = L_motor_sensor[0]->getValue();
	X_sensor[4] = L_motor_sensor[1]->getValue();
	X_sensor[5] = L_motor_sensor[2]->getValue();

	Imu_value[0] = vg_value[0];
	Imu_value[1] = vg_value[1];
	Imu_value[2] = vg_value[2];

	Gps_value[0] = -gps_value[2];
	Gps_value[1] = gps_value[0];
	Gps_value[2] = gps_value[1];

	for (int i = 0;i < 6 ; i++)
	{
		if(StarTime==true)				//防止仿真开始瞬间力矩过大
			X_sensor_e[i] = X_sensor[i];
		//cout << "current: " << X_sensor[i] << "old: " << X_sensor_e [i]<< endl;
		dX_sensor[i] = (X_sensor[i] - X_sensor_e[i]) / (timeStep / 1000.0);
		X_sensor_e[i] = X_sensor[i];
		dX_sensor[i] = Clamp(dX_sensor[i], -5, 5);
		//cout << "x_sensor_e:" << X_sensor_e[i] << endl;
	}
	for (int i = 0;i < 3; i++)
	{
		if (StarTime == true)				//防止仿真开始瞬间力矩过大
			Imu_value_e[i] = Imu_value[i];
		dImu_value[i] = (Imu_value[i] - Imu_value_e[i]) / (timeStep / 1000.0);
		Imu_value_e[i] = Imu_value[i];
		dImu_value[i] = Clamp(dImu_value[i], -5, 5);
	}
	for (int i = 0; i < 3; i++)
	{
		if (StarTime == true)				//防止仿真开始瞬间力矩过大
			Gps_value_e[i] = Gps_value[i];
		dGps_value[i] = (Gps_value[i] - Gps_value_e[i]) / (timeStep / 1000.0);
		Gps_value_e[i] = Gps_value[i];
		dGps_value[i] = Clamp(dGps_value[i], -5, 5);
	}
	StarTime = false;
}

