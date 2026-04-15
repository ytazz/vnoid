#pragma once

#include "../src/robot.h"

#include <cnoid/Joystick>

/*
 * Tutorial 1
 * 
 * Use this code template when you go through this tutorial.
 * see myrobot_complete.[h|cpp] for complete code.
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

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    void Visualize();
	
	MyRobot();

};

}
}