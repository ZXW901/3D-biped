#ifndef CLASSES_H_
#define CLASSES_H_

#include <math.h>
#include <iostream>


class Cartesian
{
public:
	Cartesian();
	Cartesian(double in1, double in2, double in3); //���캯����ֱ������xyz����

	~Cartesian();
	
	double x;
	double y;
	double z;

	Cartesian operator+(Cartesian);              //�������
	Cartesian operator-(Cartesian);              //�������
	Cartesian operator/(int);					 //����任   

};


class Angle
{
public:
	Angle();
	Angle(double in1, double in2, double in3);     //���캯����ֱ�Ӹ��������Ƕ�
	~Angle();

	double theta[3];

	Angle operator+(Angle);                      //���ؼӷ������
	Angle operator-(Angle);

};


class XYZ                                        //X-Y-Zŷ����
{
public:
	double roll;                                 //�����
	double PItch;                                //������
	double yaw;                                  //�����
	XYZ();
	XYZ(double r, double p, double y);
	Cartesian operator*(Cartesian);              //������������ת
};

class ZYX                                        //Z-Y-Xŷ����
{
public:
	double yaw;
	double PItch;
	double roll;
	ZYX();
	ZYX(double y, double p, double r);
};

class Rotationmatrix                             //��ת����
{
public:
	double rot[3][3];                            //3*3Ԫ��
	Rotationmatrix();
	Rotationmatrix(XYZ);                         //ʹ��X-Y-Zŷ���ǹ�����ת����
	Rotationmatrix(ZYX);                         //ʹ��Z-Y-Xŷ���ǹ�����ת����
	Cartesian operator*(Cartesian);              //������������ת
	Rotationmatrix operator*(Rotationmatrix);    //����˷�
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

