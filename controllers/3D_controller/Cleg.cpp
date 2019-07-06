#include "Cleg.h"

using namespace std;

Cleg::Cleg()
{
	name = 0;
}

Cleg::~Cleg()
{
}

Cartesian  Cleg::Angle2Hip(Angle ang)
{
	if (isnan(ang.theta[0]) || isnan(ang.theta[1]) || isnan(ang.theta[2]))
	{
		cout << "CLeg::Angle2Hip ang NaN!Leg:" << name << endl;
		return Cartesian(0, 0, 0);
	}
	Cartesian hip;
	hip.x = -L1 * sin(ang.theta[1]) - L2 * sin(ang.theta[1] + ang.theta[2]);
	if (name == 'R')	//右腿
	{
		hip.y = L0 * sin(ang.theta[0]) + L1 * sin(ang.theta[0])*cos(ang.theta[1]) + L2 * sin(ang.theta[0])*cos(ang.theta[1] + ang.theta[2]) - 1000 * Y0_OFFSET * cos(ang.theta[0]);
		hip.z = -L0 * cos(ang.theta[0]) - L1 * cos(ang.theta[0])*cos(ang.theta[1]) - L2 * cos(ang.theta[0])*cos(ang.theta[1] + ang.theta[2]) - 1000 * Y0_OFFSET * sin(ang.theta[0]);
	}
	else if (name == 'L')
	{
		hip.y = L0 * sin(ang.theta[0]) + L1 * sin(ang.theta[0])*cos(ang.theta[1]) + L2 * sin(ang.theta[0])*cos(ang.theta[1] + ang.theta[2]) + 1000 * Y0_OFFSET * cos(ang.theta[0]);
		hip.z = -L0 * cos(ang.theta[0]) - L1 * cos(ang.theta[0])*cos(ang.theta[1]) - L2 * cos(ang.theta[0])*cos(ang.theta[1] + ang.theta[2]) + 1000 * Y0_OFFSET * sin(ang.theta[0]);
	}
	else
		cout << "Controller::Angle2Hip Leg Name Error!" << endl;
	return hip;
}

Angle Cleg::Hip2Angle(Cartesian hip)
{
	if (isnan(hip.x) || isnan(hip.y) || isnan(hip.z))
	{
		cout << "CLeg::Hip2Angle hip NaN!Leg:" << name << endl;
		return Angle(0, 0, 0);
	}
	Angle ang;
	double fai;
	double L12 = 550;    //1号关节到足端的长度
	static double L12_min, L12_max, theta0_min, theta0_max, theta1_min, theta1_max, theta2_min, theta2_max;
	static int physical_constrain = 0; //是不是对机器人的关节按照实际物理设计来约束
	if (physical_constrain)
	{
		L12 = 800;    //1号关节到足端的长度
		L12_min = 253.0992125693093;//这是计算值，测量值是245
		L12_max = 832.5684843904218;
		theta0_min = -0.710716606163152;
		theta0_max = 0.364826311943780;
		theta1_min = -2.032361932662128;   //-2.0316
		theta1_max = 0.047386828976372;    //0.0471
		theta2_min = 0.491680761417470;    //0.4503   足底坐标点不一样，这是足底，我是足中心
		theta2_max = 2.570257801452994;    //2.5290
	}
	else
	{
		L12 = 800;    //1号关节到足端的长度
		L12_min = 10;// 253.0992125693093;//这是计算值，测量值是245
		L12_max = 1000;// 832.5684843904218;
		theta0_min = -1;// -0.710716606163152;
		theta0_max = 1;// 0.364826311943780;
		theta1_min = -3;// -2.032361932662128;   //-2.0316
		theta1_max = 3;// 0.047386828976372;    //0.0471
		theta2_min = -3;// 0.491680761417470;    //0.4503   足底坐标点不一样，这是足底，我是足中心
		theta2_max = 3;// 2.570257801452994;    //2.5290

	}

	if (name == 'R')
	{
		if (hip.y < 0)
			ang.theta[0] = asin((-2 * 1000 * Y0_OFFSET * hip.z - sqrt(2 * 2 * 1000 * Y0_OFFSET * 1000 * Y0_OFFSET * hip.z*hip.z - 2 * 2 * (hip.y*hip.y + hip.z*hip.z)*(1000 * Y0_OFFSET * 1000 * Y0_OFFSET - hip.y*hip.y))) / 2.0 / (hip.y*hip.y + hip.z*hip.z));
		else
			ang.theta[0] = asin((-2 * 1000 * Y0_OFFSET * hip.z + sqrt(2 * 2 * 1000 * Y0_OFFSET * 1000 * Y0_OFFSET * hip.z*hip.z - 2 * 2 * (hip.y*hip.y + hip.z*hip.z)*(1000 * Y0_OFFSET * 1000 * Y0_OFFSET - hip.y*hip.y))) / 2.0 / (hip.y*hip.y + hip.z*hip.z));
		ang.theta[0] = Clamp(ang.theta[0], theta0_min, theta0_max);
	}
	else if (name == 'L')
	{
		if (hip.y < 0)
			ang.theta[0] = asin((2 * 1000 * Y0_OFFSET * hip.z - sqrt(2 * 2 * 1000 * Y0_OFFSET * 1000 * Y0_OFFSET * hip.z*hip.z - 2 * 2 * (hip.y*hip.y + hip.z*hip.z)*(1000 * Y0_OFFSET * 1000 * Y0_OFFSET - hip.y*hip.y))) / 2.0 / (hip.y*hip.y + hip.z*hip.z));
		else
			ang.theta[0] = asin((2 * 1000 * Y0_OFFSET * hip.z + sqrt(2 * 2 * 1000 * Y0_OFFSET * 1000 * Y0_OFFSET * hip.z*hip.z - 2 * 2 * (hip.y*hip.y + hip.z*hip.z)*(1000 * Y0_OFFSET * 1000 * Y0_OFFSET - hip.y*hip.y))) / 2.0 / (hip.y*hip.y + hip.z*hip.z));
		ang.theta[0] = Clamp(ang.theta[0], -theta0_max, -theta0_min);
	}
	else
		cout << "Controller::Hip2Angle leg name error!" << endl;

	double delta = 0;
	if (name == 'R')
		delta = 1;
	else if (name == 'L')
		delta = -1;

	L12 = sqrt((hip.y + delta * 1000 * Y0_OFFSET * cos(ang.theta[0]) - L0 * sin(ang.theta[0]))*(hip.y + delta * 1000 * Y0_OFFSET * cos(ang.theta[0]) - L0 * sin(ang.theta[0])) + (-hip.z - delta * 1000 * Y0_OFFSET * sin(ang.theta[0]) - L0 * cos(ang.theta[0]))*(-hip.z - delta * 1000 * Y0_OFFSET * sin(ang.theta[0]) - L0 * cos(ang.theta[0])) + hip.x*hip.x);
	L12 = Clamp(L12, L12_min, L12_max);
	//cout << "L12: " << L12 << endl;

	if (L12>L12_max)
		cout << name <<"L12 larger than max_L12" << endl;
	else if (L12<L12_min)
		cout << name <<"L12 smaller than min_L12" << endl;
	L12 = Clamp(L12, L12_min, L12_max);
	double abs_fai;                              //发现有时候这个计算出来的acos里面的值会超过1 所以限制一下
	if ((L1*L1 + L12 * L12 - L2 * L2) / 2.0 / L1 / L12>1)
		abs_fai = 1;
	else if ((L1*L1 + L12 * L12 - L2 * L2) / 2.0 / L1 / L12 < -1)
		abs_fai = -1;
	else
		abs_fai = (L1*L1 + L12 * L12 - L2 * L2) / 2.0 / L1 / L12;
	fai = acos(abs_fai);

	ang.theta[1] = atan2(-hip.x, (-hip.z - delta * 1000 * Y0_OFFSET * sin(ang.theta[0]) - L0 * cos(ang.theta[0])) / cos(ang.theta[0])) + fai;
	//cout << "theta1: " << ang.theta[1] << endl;
	if (ang.theta[1] >-theta1_min)
		cout << name << "theta1 larger than max_angle" << endl;
	else if (ang.theta[1]< -theta1_max)
		cout << name <<"theta1 smaller than min_angle" << endl;
	ang.theta[1] = Clamp(ang.theta[1], -theta1_max, -theta1_min);


	if ((L1*L1 + L12 * L12 - L2 * L2) / 2.0 / L1 / L12>1)
		ang.theta[2] = -PI + acos(1);
	else if ((L1*L1 + L12 * L12 - L2 * L2) / 2.0 / L1 / L12 < -1)
		ang.theta[2] = -PI + acos(-1);
	else
		ang.theta[2] = -PI + acos((L1*L1 + L2 * L2 - L12 * L12) / 2.0 / L1 / L2);

	ang.theta[2] = Clamp(ang.theta[2], -theta2_max, -theta2_min);


	if (isnan(ang.theta[0]) || isnan(ang.theta[1]) || isnan(ang.theta[2]))
	{
		cout << "Controller::hip is correct but calculated ang is nan" << ang.theta[0] << "\t" << ang.theta[1] << "\t" << ang.theta[2] << "\t" << name << endl;
		cout << "iniput Cartesian: " << hip.x << "\t" << hip.y << "\t" << hip.z << endl;
		//ang.theta[0]
	}

	return ang;
}

Torque Cleg::LegForce2Torque(Force force)
{
	if (isnan(force.force[0]) || isnan(force.force[1]) || isnan(force.force[2]))
	{
		cout << "CLeg::LegForce2Torque force NaN!Leg:" << name << endl;
		return Torque(0, 0, 0);
	}
	Torque torque;
	Angle ang = currentang;
	double d2 = 0;
	double t0 = ang.theta[0];
	double t1 = ang.theta[1];
	double t2 = ang.theta[2];
	if (name == 'R')
		d2 = -100.0;
	else if (name == 'L') 
		d2 = 100.0;
	else
		cout << "CLeg::LegForce2Torque leg name error!" << endl;
	torque.torque[0] = (L0*cos(t0) + L1 * cos(t0)*cos(t1) + L2 * cos(t0)*cos(t1 + t2) - d2 * sin(t0))*force.force[1] + (L0*sin(t0) + L1 * sin(t0)*cos(t1) + L2 * sin(t0)*cos(t1 + t2) + d2 * cos(t0))*force.force[2];
	torque.torque[1] = (-L1 * cos(t1) - L2 * cos(t1 + t2))*force.force[0] + (-L1 * sin(t0)*sin(t1) - L2 * sin(t0)*sin(t1 + t2))*force.force[1] + (L1*cos(t0)*sin(t1) + L2 * cos(t0)*sin(t1 + t2))*force.force[2];
	torque.torque[2] = (-L2 * cos(t1 + t2))*force.force[0] + (-L2 * sin(t0)*sin(t1 + t2))*force.force[1] + (L2*cos(t0)*sin(t1 + t2))*force.force[2];
	torque.torque[0] *= 0.001;
	torque.torque[1] *= 0.001;
	torque.torque[2] *= 0.001;
	return torque;
}
