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
    bool           use_joystick;
    Joystick       joystick;

	Timer          timer;
    Param          param;
    Base           base;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;
    
    IkSolver       ik_solver;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}