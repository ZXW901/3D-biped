#include "Controller.h"

using namespace std;

Controller::Controller()
{
	leg[0].name = 'L';
	leg[1].name = 'R';
	u[6] = 0;
	dt = crobot.timeStep / 1000.0;
	T = 0;
	t = 0;
	state = 1;
	stanceLeg = 1;
	x_st_e = 0, x_sw_e = 0;
	x_est = 0, y_est = 0;
	dx_est = 0, dy_est = 0, dx_est_e = 0, dy_est_e = 0;
	dx_tgt = 0, dy_tgt = 0;
	dx_cmd = 0, dy_cmd = 0;
	dTarget_angle[6] = { 0 };
	Target_angle_e[6] = { 0 };
	hipOffset.y = 100;	//单位毫米
	stanceAngle = 0;
}

Controller::~Controller()
{
}

Angle Controller::CalcStlegAngle(double L, double X, double phi)
{
	Angle tmp;
	double c = pow(L1/1000.0, 2) - pow((L / 2.0), 2);
	if (c < 0)
	{
		c = 0;
		cout << "CalcStlegAngle sqrt <0" << endl;
	}
	double a = sqrt(c);
	double A = asin(a / (L1/1000.0));
	double B = asin(X / L);
	tmp.theta[1] = A - B + phi;
	tmp.theta[2] = -2 * A;
	//cout << "a: " << a << " A: " << A << " B: " << B << " X: " << X << " phi: " << phi;
	return tmp;
}

BodyVelocity Controller::CalBodyVel(double x, double y)
{
	BodyVelocity Velocity;
	static double x_e = 0, y_e = 0;
	static double dx = 0, dy = 0;
	double dxC = (x - x_e) / dt;
 	double dyC = (y - y_e) / dt;
	x_e = x;
	y_e = y;
	//dx = dx + 0.12 * (dxC - dx)*(abs(dxC) < 3);
	//dy = dy + 1 * (dyC - dy)*(abs(dyC) < 1);
	Velocity.dx = dxC;
	Velocity.dy = dyC;
	return Velocity;
}

void Controller::GetKeyBoard(int key)
{
	static int key_pro = -1;
	if (key != key_pro)
	{
		//cout << key << endl;
		switch (key)
		{
		case 314:
			dy_cmd -= 0.1;
			break;
		case 315:
			dx_cmd += 0.4;
			break;
		case 316:
			dy_cmd += 0.1;
			break;
		case 317:
			dx_cmd -= 0.4;
			break;
		case 32:
			state = 0;
			break;
		default:
			break;
		}
	}
	dx_cmd > 1.2 ? dx_cmd = 1.2 :dx_cmd=dx_cmd;
	dx_cmd < -1.2 ? dx_cmd = -1.2 : dx_cmd = dx_cmd;
	dy_cmd > 0.3 ? dy_cmd = 0.3 : dy_cmd = dy_cmd;
	dy_cmd < -0.3 ? dy_cmd = -0.3 : dy_cmd = dy_cmd;
	key_pro = key;
}

int Controller::sign(double x)
{
	if ( x > 0 )
		return 1;
	else if ( x < 0 )
		return -1;
	else
		return 0;
}

void Controller::Log(double x1, double x2)
{
	oFile << x1 << "," << x2 << endl;
}

void Controller::Log(double x1, double x2, double x3, double x4)
{
	oFile << x1 << "," << x2 << "," << x3 << "," << x4 << endl;
}

void Controller::run()
{
	T = T + dt;						//系统时间
	double x_st, x_sw, y_st, y_sw;	//支撑腿相对于髋关节在x方向的位移，摆动腿……
									//支撑腿相对于髋关节在y方向的位移, 摆动腿……
	double l_t = 0.335*M_LEG / (M_TORSO + 2 * M_LEG);	//不知道是啥？？？
	int leg_f[2];
	int leg_u[4];
	int hip_u[2];
	char flag_leg[2] = { 'L','R' };
	xyz.roll = crobot.Imu_value[1];
	xyz.PItch = crobot.Imu_value[0];
	xyz.yaw = crobot.Imu_value[2];
	Rotationmatrix R(xyz);			//旋转矩阵

	if (stanceLeg == 1)		//左腿支撑
	{
		leg_f[0] = 0;
		leg_f[1] = 1;

		leg_u[0] = 4;
		leg_u[1] = 5;
		leg_u[2] = 1;
		leg_u[3] = 2;

		hip_u[0] = 3;
		hip_u[1] = 0;
	}
	else					//右腿支撑
	{
		leg_f[0] = 1;
		leg_f[1] = 0;

		leg_u[0] = 1;
		leg_u[1] = 2;
		leg_u[2] = 4;
		leg_u[3] = 5;

		hip_u[0] = 0;
		hip_u[1] = 3;
	}

	bool connect = crobot.JoyStick.isConnected();
	if (connect == 1)		//手柄
	{
		string mod = crobot.JoyStick.getModel();
		int X_AxisValue = crobot.JoyStick.getAxisValue(0);
		int Y_AxisValue = crobot.JoyStick.getAxisValue(3);
		int stop = crobot.JoyStick.getPressedButton();
		dx_cmd = -1.2 / 32767 * X_AxisValue;
		dy_cmd = 0.3 / 32767 * Y_AxisValue;
		if (dy_cmd > 0 && dy_cmd < 0.03) dy_cmd = 0;
		if (dy_cmd < 0 && dy_cmd > -0.03) dy_cmd = 0;
		state = -stop;
	}
	else            //键盘
	{
		key = crobot.KeyBoard.getKey();	
		GetKeyBoard(key);		
	}

	//cout << Y_AxisValue << endl;
	//cout << mod << endl;
	//cout << "connect: " << connect << endl;
	switch (state)
	{
	case 1:		//正常运行
	{
		gravitycom = leg[leg_f[0]].LegForce2Torque(gravity);  //支撑腿重力补偿

		double alpha1 = dt / (3 + dt);	//平滑因子
		dx_tgt = dx_tgt + alpha1 * (dx_cmd - dx_tgt); //目标速度滤波
		dy_tgt = dy_tgt + alpha1 * (dy_cmd - dy_tgt);

		t = t + dt;		//腿切换时间
		for (int i = 0; i < 3; i++)		//当前各关节角度
		{
			leg[1].currentang.theta[i] = crobot.X_sensor[i];
			leg[0].currentang.theta[i] = crobot.X_sensor[i + 3];
		}
		leg[0].hipCar = leg[0].Angle2Hip(leg[0].currentang);	//足端相对于髋关节坐标
		leg[1].hipCar = leg[1].Angle2Hip(leg[1].currentang);
		//cout<< "y1: " << leg[1].hipCar.y << "  y2: " << leg[0].hipCar.y << endl;

		//转化到世界坐标系中（先转到基体坐标系，再由基体坐标系转化到世界坐标系）
		leg[0].bodyCar.x = leg[0].hipCar.x;
		leg[0].bodyCar.y = leg[0].hipCar.y + 100;
		leg[0].bodyCar.z = leg[0].hipCar.z;
		leg[1].bodyCar.x = leg[1].hipCar.x;
		leg[1].bodyCar.y = leg[1].hipCar.y - 100;
		leg[1].bodyCar.z = leg[1].hipCar.z;
		//cout << "body_x: " << leg[1].bodyCar.x << "  body_y: " << leg[1].bodyCar.y << "  body_z: " << leg[1].bodyCar.z << endl;
		
		//基体体坐标转化为世界坐标
		Eigen::Matrix3d R_temp;
		Eigen::Matrix3d R_inverse;
		Eigen::Vector3d leg_tmp0;
		Eigen::Vector3d leg_tmp1;
		Eigen::Vector3d leg_tmp2;
		Eigen::Vector3d leg_tmp3;

		R_temp << R.rot[0][0], R.rot[0][1], R.rot[0][2],
			R.rot[1][0], R.rot[1][1], R.rot[1][2],
			R.rot[2][0], R.rot[2][1], R.rot[2][2];
		R_inverse = R_temp.inverse();

		leg_tmp0 << leg[0].bodyCar.x, leg[0].bodyCar.y, leg[0].bodyCar.z;
		leg_tmp1 << leg[1].bodyCar.x, leg[1].bodyCar.y, leg[1].bodyCar.z;

		leg_tmp2 = R_inverse * leg_tmp0;
		leg_tmp3 = R_inverse * leg_tmp1;
		//cout << leg_tmp2<<endl;
		//cout << "world_x: " << leg_tmp2[0] << "  world_y: " << leg_tmp2[1] << "  world_z: " << leg_tmp2[2] << endl;

		if (stanceLeg == 1)		//支撑腿相对于髋关节在x(y)方向的位移，摆动腿……
		{
			//x_st = leg[0].bodyCar.x / 1000.0;
			x_st = leg_tmp2[0] / 1000.0;		//单位转化为m
			y_st = leg_tmp2[1] / 1000.0;
			x_sw = leg_tmp3[0] / 1000.0;
			y_sw = leg_tmp3[1] / 1000.0;
		}
		else
		{
			//x_st = leg[1].bodyCar.x / 1000.0;
			x_st = leg_tmp3[0] / 1000.0;
			y_st = leg_tmp3[1] / 1000.0;
			x_sw = leg_tmp2[0] / 1000.0;
			y_sw = leg_tmp2[1] / 1000.0;
		}
		//cout << "x_st:" << x_st * 1000 << "  " << "y_st:" << y_st * 1000 << "  " << "x_sw" << x_sw * 1000 << "  " << "y_sw:" << y_sw * 1000 << endl;
		//cout << "y_st: " << y_st << endl;
		//Scaling factors representing the magnitude of force in each leg
		double s_st = scaleFactor(crobot.F_touch[leg_f[0]], THRES_LO, THRES_HI);		//具体范围怎么设置？（10，50）（50，100）
		double s_sw = scaleFactor(crobot.F_touch[leg_f[1]], THRES_LO, THRES_HI);
		//cout << "s_st:" << s_st << "  "<<"s_sw:" << s_sw << endl;

		//质心在x和y方向的速度
		BodyVel = CalBodyVel(x_st, y_st);

		double dx = crobot.dGps_value[0];
		double dy = crobot.dGps_value[1];
		//cout << "dx: " << dx1 << "  gps: " << dx << endl;
		//cout << "dy: " << dy1 << "  gps: " << dy << endl;
		if (s_st >= 1)	//速度滤波
		{
			double alpha = dt / (0.12 + dt);
			double alpha2 = dt / (0.12 + dt);
			dx_est = dx_est + alpha * (dx - dx_est)*(abs(dx) < 3);
			dy_est = dy_est + alpha2*(dy - dy_est)*(abs(dy) < 1);
		}

		cout << "dx_cmd: " << dx_cmd << "  dy_cmd: " << dy_cmd << "  dx_est: " << dx_est << "  dy_est: " << dy_est << endl;
	Log(dx_est, dx_est_real); //日记
		//质心在x和y方向的位移
		x_est = x_est + dx_est * dt;
		y_est = y_est + dy_est * dt;

		//Stance leg push-off is proportional to desired speed and error				
		double l_ext = Clamp(L_EXT_GAIN*abs(dx_tgt), 0, 0.86 - L0_LEG)*(sign(dx_tgt) == sign(dx_est));

		//Step length is proportional to current velocity           这有问题!!!!!!!!
		double l_step = Clamp(dx_est * 0.18+ (dx_est - dx_tgt)*0.1 + (dx_est - dx_est_e)*0.1 + l_t * sin(crobot.Imu_value[0]), -0.4, 0.4);
		//cout << "step: " << l_step << "  realStep: " << x_sw << endl;

		double s = Clamp(t / T_STEP, 0, 1);		//Define a time variant parameter

		//摆动腿的z方向收缩量
		if (s < 0.5)
			l_sw = Cubic(0, 0.5, L0_LEG, L0_LEG - L_RET, 0, 0, s, 1);
		else
			l_sw = Cubic(0.5, 1, L0_LEG - L_RET, L0_LEG, 0, 0, s, 1);

		//摆动腿的x方向位移
		d_sw = Cubic(0, 0.7, x_sw_e, l_step, 0, 0, s, 1);
		//cout << "d_sw:"<< d_sw.y << endl;

		//摆动腿的y方向位移
		double d = -(Y0_OFFSET - Y0_GAIN * abs(dx_tgt))*stanceLeg -
			dy_est * 0.19 -
					(dy_est - dy_tgt)*0.1 -
					(dy_est - dy_est_e)*0.1 +
					l_t * sin(crobot.Imu_value[1]);
		//cout << "d: " << d * 1000 << "	k1: " << dy_est << "    k2: " << dy_est - dy_tgt << "	k3: " << dy_est - dy_est_e << endl;
		//cout << "dy: " << dy_est << endl;

		//支撑腿的z方向伸长量
		double l_st = L0_LEG +l_ext * Clamp(2 * s - 1, 0, 1);
		double dl_st = l_ext / (T_STEP / 2);

		//世界坐标系中的足端轨迹规划(所有的轨迹规划均在世界坐标系中)

		//摆动腿
		leg[leg_f[1]].worldCar.x = d_sw.y * 1000;
		leg[leg_f[1]].worldCar.y = d * 1000;
		leg[leg_f[1]].worldCar.z = -l_sw.y * 1000;

		//由世界坐标系转化到基体坐标系
		leg[0].bodyCar = R * leg[0].worldCar;
		leg[1].bodyCar = R * leg[1].worldCar;
		//由基体坐标系转化到髋坐标系
		leg[0].hipCar = leg[0].bodyCar - hipOffset;
		leg[1].hipCar = leg[1].bodyCar + hipOffset;

		Target_angle[leg_f[0]] = CalcStlegAngle(l_st, x_st, xyz.PItch);				//支撑腿各关节期望角度
		Target_angle[leg_f[1]] = leg[leg_f[1]].Hip2Angle(leg[leg_f[1]].hipCar);		//摆动腿各关节期望角度

		//髋关节重力补偿
		if (s_st > 0 || s_sw > 0)
		{
			u[hip_u[0]] = 3 * G * Y0_OFFSET * stanceLeg;
			u[hip_u[1]] = 3 * G * Y0_OFFSET * (-stanceLeg);
		}

		for (int i = 0; i < 3; i++)				//角速度(摆动期)
		{
			dTarget_angle[i] = (Target_angle[leg_f[1]].theta[i] - Target_angle_e[i]) / dt;
			Target_angle_e[i] = Target_angle[leg_f[1]].theta[i];
			dTarget_angle[i] = Clamp(dTarget_angle[i], -5, 5);
		}

		for (int i = 3; i < 6; i++)				//角速度(支撑期)
		{
			dTarget_angle[i] = (Target_angle[leg_f[0]].theta[i - 3] - Target_angle_e[i]) / dt;
			Target_angle_e[i] = Target_angle[leg_f[0]].theta[i - 3];
			dTarget_angle[i] = Clamp(dTarget_angle[i], -5, 5);
		}
		//摆动腿roll扭矩
		u[hip_u[1]] = u[hip_u[1]] + s * (Target_angle[leg_f[1]].theta[0] - crobot.X_sensor[hip_u[1]]) * 5000 + (dTarget_angle[0] - crobot.dX_sensor[hip_u[1]]) * 50;
		//摆动腿thigh扭矩
		u[leg_u[2]] = (Target_angle[leg_f[1]].theta[1] - crobot.X_sensor[leg_u[2]]) * 300 + (dTarget_angle[1] - crobot.dX_sensor[leg_u[2]]) * 20;//+(sin(leg[leg_f[1]].currentang.theta[1])*0.25 + sin(leg[leg_f[1]].currentang.theta[1])*0.5 + sin(leg[leg_f[1]].currentang.theta[1] + leg[leg_f[1]].currentang.theta[2] + PI / 2.0)*0.25) / 2.0 * 3 * G;
		//摆动腿shin扭矩
		u[leg_u[3]] = (Target_angle[leg_f[1]].theta[2] - crobot.X_sensor[leg_u[3]]) * 600 + (dTarget_angle[2] - crobot.dX_sensor[leg_u[3]]) * 30+sin((leg[leg_f[1]].currentang.theta[1] - leg[leg_f[1]].currentang.theta[2]) - PI / 2.0)*0.25 * 1.5 * G;
		//cout << "target1: " << Target_angle[leg_f[1]].theta[1] << "  est1: " << crobot.X_sensor[leg_u[2]] << "  target2: " << Target_angle[leg_f[1]].theta[2] << " est2:  " << crobot.X_sensor[leg_u[3]] << endl;

		//支撑腿thigh扭矩
		u[leg_u[0]] = (Target_angle[leg_f[0]].theta[1] - crobot.X_sensor[leg_u[0]])*0 + (dTarget_angle[4] - crobot.dX_sensor[leg_u[0]])*0;
		//支撑腿shin扭矩
		u[leg_u[1]] = (Target_angle[leg_f[0]].theta[2] - crobot.X_sensor[leg_u[1]]) * 8000 + (dTarget_angle[5] - crobot.dX_sensor[leg_u[1]]) * 60 - gravitycom.torque[2];
		


		//Add additional torque commands to leg actuators to stabilize torso
		u[leg_u[0]] = u[leg_u[0]] + (crobot.Imu_value[0] * -6000 + crobot.dImu_value[0] * -100);

		//double arg = 0.00;
		//if (stanceLeg == -1)
		//	arg = -0.00;
		//if (s < 0.5)
		//	hip_arg = Cubic(0, 0.5, stanceAngle, arg, 0, 0, s, 1);
		//else
		//	hip_arg = Cubic(0.5, 1, arg, 0, 0, 0, s, 1);
		//cout << hip_arg.y << endl;

		//Torso stabilization weighted PD controller
		u[hip_u[0]] = u[hip_u[0]] + s_st * S_TORSO*((0-crobot.Imu_value[1]) * 10000 + crobot.dImu_value[1] * -150);
		u[hip_u[1]] = u[hip_u[1]] + s_sw * S_TORSO*(crobot.Imu_value[1] * -10000 + crobot.dImu_value[1] * -150);

		//支撑腿和摆动腿的切换
		if ((s_sw > s_st && t > 0.2) || s >= 1)
		{
			stanceLeg = -stanceLeg;

			x_st_e = x_sw;
			x_sw_e = x_st;
			dx_est_e = dx_est;
			dy_est_e = dy_est;
			stanceAngle = crobot.Imu_value[0];

			t = 0;
			x_est = 0;
			y_est = 0;
		}
		break;
	}

	default:	//停止
		t = 0; T = 0;
		x_est = 0; y_est = 0;
		dx_est = 0; dy_est = 0;
		dx_tgt = 0; dy_tgt = 0;
		x_st_e = 0; x_sw_e = 0;
		dx_est_e = 0; dy_est_e = 0;
		crobot.JoyStick.disable();
		crobot.KeyBoard.disable();

		u[0] = (0 - crobot.dX_sensor[0])*KD_HIP;
		u[3] = (0 - crobot.dX_sensor[3])*KD_HIP;

		u[1] = (0 - crobot.dX_sensor[1])*KD_LEG;
		u[2] = (0 - crobot.dX_sensor[2])*KD_LEG;

		u[4] = (0 - crobot.dX_sensor[4])*KD_LEG;
		u[5] = (0 - crobot.dX_sensor[5])*KD_LEG;
		break;
	}

	for (int i = 0; i < 6; i++)		//力矩限制
	{
		u[i] = Clamp(u[i], -U_LIM, U_LIM);
	}

	crobot.R_motor[0]->setTorque(u[0]);
	crobot.R_motor[1]->setTorque(u[1]);
	crobot.R_motor[2]->setTorque(u[2]);
	
	crobot.L_motor[0]->setTorque(u[3]);
	crobot.L_motor[1]->setTorque(u[4]);
	crobot.L_motor[2]->setTorque(u[5]);
}
