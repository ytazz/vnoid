#pragma once

#include "../src/robot_demo.h"
#include "fksolver.h"
#include "iksolver.h"

/*
 * Inverse kinematics demo2  CoM IK
 * 
 * 
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public RobotDemo{
public:
	Timer          timer;
    Param          param;
    Base           base;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;
    
    FkSolver       fk_solver;
    IkSolver       ik_solver;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}