#pragma once

#include "robot.h"
#include "footstep.h"
#include "path.h"
#include "pathplanner.h"
#include "planner_capt.h"
#include "planner_simple.h"
#include "iksolver_sb.h"
#include "bridge_udp.h"

namespace cnoid{
namespace BipedCnoid{

class MyRobot : public Robot{
public:
    // robot setting
    Obstacles*    obstacles;
	Path*         path;
    
    PathPlanner*  pathPlanner;
    PlannerCapt*  plannerCapt;

    real_t        planPeriod;
	int           index;
    real_t        minStepDuration;
    real_t        spacing;

    real_t        lastPlanTime;

    // obstacle setting
    PlannerSimple*  plannerSimple;
	
public:
	virtual void  Read   (XMLNode* node);
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot(int idx);

};

}
}