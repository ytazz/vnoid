#pragma once

#include "robot.h"
#include "iksolver.h"
#include "footstep.h"
#include "footstep_planner.h"

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
    double    standby_period;      ///< period of initial standby mode
	double    standby_com_height;  ///< com height in standby mode

    double    planPeriod;
	double    minStepDuration;
    double    spacing;

    double    lastPlanTime;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}