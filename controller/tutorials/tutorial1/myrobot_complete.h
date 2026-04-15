#pragma once

#include "../src/robot.h"
#include "fksolver.h"
#include "iksolver.h"
#include "stabilizer.h"
#include "footstep_planner.h"
#include "stepping_controller.h"
#include "visualizer.h"

#include <cnoid/Joystick>

/*
 * Tutorial 1
 * 
 * Use this code when you go through this tutorial.
 * 
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

    Joystick       joystick;

    FkSolver       fk_solver;
    IkSolver       ik_solver;

    Stabilizer     stabilizer;

    Visualizer     visualizer;

    Footstep            footstep;    
    Footstep            footstep_buffer;
    FootstepPlanner     footstep_planner;
    SteppingController  stepping_controller;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    void Visualize();
	
	MyRobot();

};

}
}