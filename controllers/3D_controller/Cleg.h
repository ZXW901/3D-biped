#ifndef CLEG_H_
#define CLEG_H_

#include "Classes.h"
#include <iostream>
#include "Config.h"

class Cleg
{
public:
	Cleg();
	~Cleg();


	Cartesian Angle2Hip(Angle);
	Angle Hip2Angle(Cartesian);
	Torque LegForce2Torque(Force force);

	//Cartesian L_foot = Cartesian('L');
	//Cartesian R_foot = Cartesian('R');
	//Angle L_angle = Angle('L');
	//Angle R_angle = Angle('R');
	Cartesian hipCar;
	Cartesian bodyCar;
	Cartesian worldCar;

	Angle angle;
	Angle currentang;
	Cartesian currentCar;
	char name;

};


#endif // !CLEG_H_
