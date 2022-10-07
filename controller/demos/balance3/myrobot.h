#pragma once

#include "robot.h"
#include "fksolver.h"
#include "iksolver.h"
#include "footstep.h"
#include "footstep_planner.h"
#include "stabilizer.h"
#include "stepping_controller.h"

/*
 * Balance control demo 3
 * 
 * Robot keeps balance on a moving floor while making steps
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
	Timer          timer;
    Param          param;
    Base           base;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;
    Footstep       footstep;
    Footstep       footstep_buffer;

    FootstepPlanner     footstep_planner;
    SteppingController  stepping_controller;
    Stabilizer          stabilizer;
    FkSolver            fk_solver;
    IkSolver            ik_solver;

    int  marker_index;
    int  num_markers;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}