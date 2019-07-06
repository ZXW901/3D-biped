// File:          3D_controller.cpp
// Date:			  2019.3.7
// Description:
// Author:
// Modifications:
#include "Controller.h"
Controller robot_con;
int main(int argc, char **argv)
{
  robot_con.oFile.open("E:\\robotData.csv", ios::out | ios::trunc);

  while (robot_con.crobot.robot->step(robot_con.crobot.timeStep) != -1) {
	  robot_con.crobot.UpdateAnalogAll(); 
	  robot_con.run();
  };
  robot_con.oFile.close();
  delete robot_con.crobot.robot;
  return 0;
}
