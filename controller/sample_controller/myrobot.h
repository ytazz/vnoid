#pragma once

#include "robot.h"
#include "iksolver.h"
#include "footstep.h"
#include "footstep_planner.h"

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
    double        planPeriod;
	double        minStepDuration;
    double        spacing;

    double        lastPlanTime;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}