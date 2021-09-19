#include <cnoid/SimpleController>
#include <cnoid/Body>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "myrobot.h"

using namespace cnoid;
using namespace cnoid::BipedCnoid;

const char* confFilename = "../share/project/BipedAvoid.conf.xml";

class BipedAvoid : public SimpleController{
public:
	MyRobot*  robot;
    int       robotIndex;  //< 0: robot,  1,2,3...: obstacle

public:
    virtual bool configure(SimpleControllerConfig* config){
        string opt = config->optionString();
        Converter::FromString(opt, robotIndex);

        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
		XML xml;
		xml.Load(confFilename);

		robot = new MyRobot(robotIndex);
		robot->Read(xml.GetRootNode()->GetNode("robot", robotIndex));
		robot->Init(io);

		return true;
	}

	virtual bool control()	{
		robot->Sense  ();
		robot->Control();
		return true;
	}
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(BipedAvoid)
