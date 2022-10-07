#pragma once

#include "robot.h"
#include "fksolver.h"
#include "iksolver.h"
#include "stabilizer.h"
#include "footstep.h"

/*
 * Balance control demo 2
 * 
 * Robot keeps balance on a tilting floor 
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
    Footstep       footstep_buffer;
    
    Stabilizer     stabilizer;
    FkSolver       fk_solver;
    IkSolver       ik_solver;

    int  marker_index;
    int  num_markers;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}