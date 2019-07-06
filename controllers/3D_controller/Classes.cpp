#include "Classes.h"

Cartesian::Cartesian()
{
	x = 0;
	y = 0;
	z = 0;

 }

Cartesian::Cartesian(double in1, double in2, double in3)
{
	x = in1;
	y = in2;
	z = in3;
}

Cartesian::~Cartesian()
{
}

Cartesian Cartesian::operator+(Cartesian c1)
{
	return Cartesian(x + c1.x, y + c1.y, z + c1.z);
}
Cartesian Cartesian::operator-(Cartesian c1)
{
	return Cartesian(x - c1.x, y - c1.y, z - c1.z);
}
Cartesian Cartesian::operator/(int c1)					 //坐标变换 
{
	if (c1 == 0)
	{
		std::cout << "Cartesian operator/0 error!!!" << std::endl;
		return Cartesian(x, y, z);
	}
	return Cartesian(x / c1, y / c1, z / c1);
}

Angle::Angle()
{
	theta[0] = 0;
	theta[1] = 0;
	theta[2] = 0;
}
Angle::Angle(double in1, double in2, double in3)
{
	theta[0] = in1;
	theta[1] = in2;
	theta[2] = in3;
}
Angle Angle::operator+(Angle a1)
{
	return Angle(theta[0] + a1.theta[0], theta[1] + a1.theta[1], theta[2] + a1.theta[2]);
}
Angle Angle::operator-(Angle a1)
{
	return Angle(theta[0] - a1.theta[0], theta[1] - a1.theta[1], theta[2] - a1.theta[2]);
}
Angle::~Angle()
{
}

XYZ::XYZ()
{
	roll = 0;
	PItch = 0;
	yaw = 0;
}
XYZ::XYZ(double r, double p, double y)
{
	roll = r;
	PItch = p;
	yaw = y;
}
Cartesian XYZ::operator*(Cartesian car)			//看这里小老弟
{
	Cartesian ret;
	ret.x = car.z*sin(PItch) + car.x*cos(PItch)*cos(yaw) - car.y*cos(PItch)*sin(yaw);
	ret.y = car.x*(cos(roll)*sin(yaw) + cos(yaw)*sin(PItch)*sin(roll)) + car.y*(cos(roll)*cos(yaw) - sin(PItch)*sin(roll)*sin(yaw)) - car.z*cos(PItch)*sin(roll);
	ret.z = car.x*(sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(PItch)) + car.y*(cos(yaw)*sin(roll) + cos(roll)*sin(PItch)*sin(yaw)) + car.z*cos(PItch)*cos(roll);
	return ret;
}
ZYX::ZYX()
{
	yaw = 0;
	PItch = 0;
	roll = 0;
}
ZYX::ZYX(double y, double p, double r)
{
	yaw = y;
	PItch = p;
	roll = r;
}
Rotationmatrix::Rotationmatrix()
{
	for (int i = 0; i<3; i++)
		for (int j = 0; j<3; j++)
			rot[i][j] = 0;
}
Rotationmatrix::Rotationmatrix(XYZ xyz) //转置后的旋转矩阵，和旋转角度负的，相当于是沿着固定轴旋转后 固定坐标系中的点在旋转后的坐标系中的位置
{
	////健哥原来解析形式
	//rot[0][0] = cos(xyz.PItch)*cos(xyz.yaw);
	//rot[0][1] = -cos(xyz.PItch)*sin(xyz.yaw);
	//rot[0][2] = sin(xyz.PItch);
	//rot[1][0] = cos(xyz.roll)*sin(xyz.yaw) + cos(xyz.yaw)*sin(xyz.PItch)*sin(xyz.roll);
	//rot[1][1] = cos(xyz.roll)*cos(xyz.yaw) - sin(xyz.PItch)*sin(xyz.roll)*sin(xyz.yaw);
	//rot[1][2] = -cos(xyz.PItch)*sin(xyz.roll);
	//rot[2][0] = sin(xyz.roll)*sin(xyz.yaw) - cos(xyz.roll)*cos(xyz.yaw)*sin(xyz.PItch);
	//rot[2][1] = cos(xyz.yaw)*sin(xyz.roll) + cos(xyz.roll)*sin(xyz.PItch)*sin(xyz.yaw);
	//rot[2][2] = cos(xyz.PItch)*cos(xyz.roll);

	// 我的解析形式
	rot[0][0] = cos(xyz.PItch)*cos(xyz.yaw);
	rot[0][1] = cos(xyz.yaw)*sin(xyz.PItch)*sin(xyz.roll) - sin(xyz.yaw)*cos(xyz.roll); //-cos(xyz.PItch)*sin(xyz.yaw);
	rot[0][2] = sin(xyz.roll)*sin(xyz.yaw) + cos(xyz.roll)*cos(xyz.yaw)*sin(xyz.PItch); //sin(xyz.PItch);
	rot[1][0] = cos(xyz.PItch)*sin(xyz.yaw);//cos(xyz.roll)*sin(xyz.yaw) + cos(xyz.yaw)*sin(xyz.PItch)*sin(xyz.roll);
	rot[1][1] = cos(xyz.roll)*cos(xyz.yaw) + sin(xyz.PItch)*sin(xyz.roll)*sin(xyz.yaw);
	rot[1][2] = cos(xyz.roll)*sin(xyz.PItch)*sin(xyz.yaw) - cos(xyz.yaw)*sin(xyz.roll); //-cos(xyz.PItch)*sin(xyz.roll);
	rot[2][0] = -sin(xyz.PItch); //sin(xyz.roll)*sin(xyz.yaw) - cos(xyz.roll)*cos(xyz.yaw)*sin(xyz.PItch);
	rot[2][1] = cos(xyz.PItch)*sin(xyz.roll); //cos(xyz.yaw)*sin(xyz.roll) + cos(xyz.roll)*sin(xyz.PItch)*sin(xyz.yaw);
	rot[2][2] = cos(xyz.PItch)*cos(xyz.roll);
}
Rotationmatrix::Rotationmatrix(ZYX zyx)    //zyx欧拉角旋转矩阵
{
	//健哥原来解析形式
	//rot[0][0] = cos(zyx.PItch)*cos(zyx.yaw);
	//rot[0][1] = cos(zyx.yaw)*sin(zyx.PItch)*sin(zyx.roll) - cos(zyx.roll)*sin(zyx.yaw);
	//rot[0][2] = sin(zyx.roll)*sin(zyx.yaw) + cos(zyx.roll)*cos(zyx.yaw)*sin(zyx.PItch);
	//rot[1][0] = cos(zyx.PItch)*sin(zyx.yaw);
	//rot[1][1] = cos(zyx.roll)*cos(zyx.yaw) + sin(zyx.PItch)*sin(zyx.roll)*sin(zyx.yaw);
	//rot[1][2] = cos(zyx.roll)*sin(zyx.PItch)*sin(zyx.yaw) - cos(zyx.yaw)*sin(zyx.roll);
	//rot[2][0] = -sin(zyx.PItch);
	//rot[2][1] = cos(zyx.PItch)*sin(zyx.roll);
	//rot[2][2] = cos(zyx.PItch)*cos(zyx.roll);

	//我的解析形式
	rot[0][0] = cos(zyx.PItch)*cos(zyx.yaw);
	rot[0][1] = cos(zyx.PItch)*sin(zyx.yaw); //cos(zyx.yaw)*sin(zyx.PItch)*sin(zyx.roll) - cos(zyx.roll)*sin(zyx.yaw);
	rot[0][2] = -sin(zyx.PItch); //sin(zyx.roll)*sin(zyx.yaw) + cos(zyx.roll)*cos(zyx.yaw)*sin(zyx.PItch);
	rot[1][0] = cos(zyx.yaw)*sin(zyx.PItch)*sin(zyx.roll) - cos(zyx.roll)*sin(zyx.yaw);// cos(zyx.PItch)*sin(zyx.yaw);
	rot[1][1] = cos(zyx.roll)*cos(zyx.yaw) + sin(zyx.PItch)*sin(zyx.roll)*sin(zyx.yaw);
	rot[1][2] = cos(zyx.PItch)*sin(zyx.roll); //cos(zyx.roll)*sin(zyx.PItch)*sin(zyx.yaw) - cos(zyx.yaw)*sin(zyx.roll);
	rot[2][0] = sin(zyx.roll)*sin(zyx.yaw) + cos(zyx.roll)*cos(zyx.yaw)*sin(zyx.PItch); //-sin(zyx.PItch);
	rot[2][1] = cos(zyx.roll)*sin(zyx.PItch)*sin(zyx.yaw) - cos(zyx.yaw)*sin(zyx.roll); //cos(zyx.PItch)*sin(zyx.roll);
	rot[2][2] = cos(zyx.PItch)*cos(zyx.roll);

}
Cartesian Rotationmatrix::operator*(Cartesian car)
{
	Cartesian ret;
	ret.x = rot[0][0] * car.x + rot[0][1] * car.y + rot[0][2] * car.z;
	ret.y = rot[1][0] * car.x + rot[1][1] * car.y + rot[1][2] * car.z;
	ret.z = rot[2][0] * car.x + rot[2][1] * car.y + rot[2][2] * car.z;
	return ret;
}
Rotationmatrix Rotationmatrix::operator*(Rotationmatrix rot2)
{
	Rotationmatrix ret;
	for (int i = 0; i<3; i++)
		for (int j = 0; j<3; j++)
			ret.rot[i][j] = rot[i][0] * rot2.rot[0][j] + rot[i][1] * rot2.rot[1][j] + rot[i][2] * rot2.rot[2][j];
	return ret;
}

CubicValue::CubicValue()
{
	y = 0;
	dy = 0;
}

CubicValue::~CubicValue()
{
}

BodyVelocity::BodyVelocity()
{
	dx = 0;
	dy = 0;
}

BodyVelocity::~BodyVelocity()
{
}

Torque::Torque()
{
	torque[0] = 0;
	torque[1] = 0;
	torque[2] = 0;
}
Torque::Torque(double in1, double in2, double in3)
{
	torque[0] = in1;
	torque[1] = in2;
	torque[2] = in3;
}
Torque::Torque(double* in)
{
	torque[0] = in[0];
	torque[1] = in[1];
	torque[2] = in[2];
}
Force::Force()
{
	force[0] = 0;
	force[1] = 0;
	force[2] = 0;
}
Force::Force(double in1, double in2, double in3)
{
	force[0] = in1;
	force[1] = in2;
	force[2] = in3;
}
Force::Force(double* in)
{
	force[0] = in[0];
	force[1] = in[1];
	force[2] = in[2];
}

double Clamp(double a, double lim1, double lim2)
{
	double a_min, a_max, b;
	if (lim1 < lim2)
	{
		a_min = lim1;
		a_max = lim2;
	}
	else if (lim2 < lim1)
	{
		a_min = lim2;
		a_max = lim1;
	}
	else
		a_min = a_max = lim1;

	if (a < a_min)
		b = a_min;
	else if (a > a_max)
		b = a_max;
	else
		b = a;
	return b;
}

double scaleFactor(double f, double tl, double tu)
{
	double s = (Clamp(f, tl, tu) - tl) / (tu - tl);
	return s;
}


CubicValue Cubic(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx)
{
	x = Clamp(x, x1, x2);
	double a0 = 2 * (y1 - y2) + (dy1 + dy2)*(x2 - x1);
	double a1 = y2 - y1 - dy1 * (x2 - x1) - a0;
	double a2 = dy1 * (x2 - x1);
	double a3 = y1;
	double s = (x - x1) / (x2 - x1);
	CubicValue value;
	value.y = a0 * pow(s, 3) + a1 * pow(s, 2) + a2 * s + a3;
	value.dy = dx * (-3 * a0 * pow((x - x1), 2) / pow((x1 - x2), 3) + 2 * a1 * (x - x1) / pow((x1 - x2), 2) - a2 / (x1 - x2));
	return value;
}
