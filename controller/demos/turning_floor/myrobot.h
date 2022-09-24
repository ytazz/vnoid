#pragma once

#include "robot.h"
#include "iksolver.h"

/*
 * Inverse kinematics demo 
 * 
 * 
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
	Timer          timer;
	Base           base;
	vector<Joint>  joint;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}