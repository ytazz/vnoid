#pragma once

#include "../src/robot_demo.h"
#include "fksolver.h"
#include "iksolver.h"
#include "reactive_walking_controller.h"

/*
 * Walking control demo 1
 * 
 * Demonstration of DCM-based reactive walking control
 * For details, see:
 * Y.Tazaki: DCM Modulation: A Three-Axis Rotation Stabilization Technique for Bipedal Locomotion Control, IROS2025
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
    
    ReactiveWalkingController   controller;

    FkSolver       fk_solver;
    IkSolver       ik_solver;
    Vector3        com_pos_prev;

    double disturbance_magnitude, disturbance_angle, disturbance_duration;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}
