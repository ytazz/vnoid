#pragma once

#include "../src/robot_demo.h"
#include "fksolver.h"
#include "iksolver.h"
#include "stabilizer.h"
#include "ground_estimator.h"
#include "footstep_planner.h"
#include "footstep.h"

/*
 * Balance control demo 2
 * 
 * Robot keeps balance on a tilting floor 
 * 
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public RobotDemo{
public:
	Timer          timer;
    Param          param;
    Base           base;
    Ground         ground;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;
    Footstep       footstep_buffer;
    
    Stabilizer          stabilizer;
    GroundEstimator     ground_estimator;
    FootstepPlanner     footstep_planner;
    FkSolver            fk_solver;
    IkSolver            ik_solver;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}