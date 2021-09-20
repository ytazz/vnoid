#pragma once

#include "robot.h"

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