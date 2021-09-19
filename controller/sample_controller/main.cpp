#include <cnoid/SimpleController>
#include <cnoid/Body>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "myrobot.h"

using namespace cnoid;
using namespace cnoid::vnoid;

class VnoidSampleController : public SimpleController{
public:
	MyRobot*  robot;

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
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
