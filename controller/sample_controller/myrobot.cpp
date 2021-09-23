#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

}

void MyRobot::Init(SimpleControllerIO* io){
	Robot::Init(io);
}

void MyRobot::Control(){
	planner->FromRobot();

	if(count % planCycle == 0){
		// generate footsteps from joystick command
		if(useJoystick){
			// erase current footsteps
			while(footstep->steps.size() > 2)
				footstep->steps.pop_back();

			Footstep::Step::Spec spec;
			spec.stride   = -maxStride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
			spec.turn     = -maxTurn  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
			spec.spacing  = 0.2;
			spec.climb    = 0.0;
			spec.duration = 0.7;
			footstep->GenerateStep(spec);
			footstep->GenerateStep(spec);
			footstep->GenerateStep(spec);
			spec.stride = 0.0;
			spec.turn   = 0.0;
			footstep->GenerateStep(spec);
		}

		planner->Plan();
	}
	planner->ToRobot();

	if(count % downsample == 0){
		Save();
	}
	
	Robot::Control();
	
	Countup();
}


}
}
