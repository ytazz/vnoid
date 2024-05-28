#pragma once

#include "../src/robot_demo.h"
#include "fksolver.h"
#include "iksolver.h"
#include "footstep.h"
#include "footstep_planner.h"
#include "stabilizer.h"
#include "stepping_controller.h"
#include "visualizer.h"

/*
 * Visualization Demo 1
 * 
 * Internal states are visualized using MarkerVisualizer plugin
 * vnoid supports two ways for visualization: 1) by using marker objects, 2) by using visualization plugin.
 * Other demos use marker objects for visualization.
 * Pros and Cons:
 * - the marker object method is limited to relatively small number of markers
 * + the visualization plugin can visualize hundred of markers
 * + marker objects are saved to log data. so markers appear when log is replayed
 * - geometries visualized by the visualization plugin are not saved to log data. so they do not appear when log is replayed
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

    /// visualizer
    Visualizer     viz;

public:
    void Visualize();

	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}