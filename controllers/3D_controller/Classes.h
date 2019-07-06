#ifndef CLASSES_H_
#define CLASSES_H_

#include <math.h>
#include <iostream>


class Cartesian
{
public:
	Cartesian();
	Cartesian(double in1, double in2, double in3); //构造函数，直接输入xyz坐标

	~Cartesian();
	
	double x;
	double y;
	double z;

	Cartesian operator+(Cartesian);              //坐标相加
	Cartesian operator-(Cartesian);              //坐标相减
	Cartesian operator/(int);					 //坐标变换   

};


class Angle
{
public:
	Angle();
	Angle(double in1, double in2, double in3);     //构造函数，直接给定三个角度
	~Angle();

	double theta[3];

	Angle operator+(Angle);                      //重载加法运算符
	Angle operator-(Angle);

};


class XYZ                                        //X-Y-Z欧拉角
{
public:
	double roll;                                 //横滚角
	double PItch;                                //俯仰角
	double yaw;                                  //航向角
	XYZ();
	XYZ(double r, double p, double y);
	Cartesian operator*(Cartesian);              //用于向量的旋转
};

class ZYX                                        //Z-Y-X欧拉角
{
public:
	double yaw;
	double PItch;
	double roll;
	ZYX();
	ZYX(double y, double p, double r);
};

class Rotationmatrix                             //旋转矩阵
{
public:
	double rot[3][3];                            //3*3元素
	Rotationmatrix();
	Rotationmatrix(XYZ);                         //使用X-Y-Z欧拉角构造旋转矩阵
	Rotationmatrix(ZYX);                         //使用Z-Y-X欧拉角构造旋转矩阵
	Cartesian operator*(Cartesian);              //用于向量的旋转
	Rotationmatrix operator*(Rotationmatrix);    //矩阵乘法
};

class CubicValue
{
public:
	CubicValue();
	~CubicValue();

	double y;
	double dy;
};

class BodyVelocity
{
public:
	BodyVelocity();
	~BodyVelocity();
	
	double dx;
	double dy;
};

class Torque                                    
{
public:
	double torque[3];
	Torque();
	Torque(double in1, double in2, double in3);
	Torque(double* in);
};

class Force                                    
{
public:
	double force[3];
	Force();
	Force(double in1, double in2, double in3);
	Force(double* in);
};

double Clamp(double a, double lim1, double lim2);
double scaleFactor(double f, double tl, double tu);
CubicValue Cubic(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx);
#endif // !CLASSES_H_

