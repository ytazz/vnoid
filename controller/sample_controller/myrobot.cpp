#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

}

void MyRobot::Init(SimpleControllerIO* io){
	Robot::Init(io, timer, joint);
}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot);

	if(timer.count % plan_cycle == 0){
		// generate footsteps from joystick command
		if(use_joystick){
		    // read joystick
		    joystick.readCurrentState();

		    /* Xbox controller mapping:
			    L_STICK_H_AXIS -> L stick right
			    L_STICK_V_AXIS -> L stick down
			    R_STICK_H_AXIS -> L trigger - R trigger
			    R_STICK_V_AXIS -> R stick down
			    A_BUTTON -> A
			    B_BUTTON -> B
			    X_BUTTON -> X
			    Y_BUTTON -> Y
			    L_BUTTON -> L
			    R_BUTTON -> R
		     */
		    /*
		    DSTR << joystick.getPosition(Joystick::L_STICK_H_AXIS) << " " 
			     << joystick.getPosition(Joystick::L_STICK_V_AXIS) << " " 
			     << joystick.getPosition(Joystick::R_STICK_H_AXIS) << " " 
			     << joystick.getPosition(Joystick::R_STICK_V_AXIS) << " " 
			     << joystick.getButtonState(Joystick::A_BUTTON) << " "
			     << joystick.getButtonState(Joystick::B_BUTTON) << " "
			     << joystick.getButtonState(Joystick::X_BUTTON) << " "
			     << joystick.getButtonState(Joystick::Y_BUTTON) << " "
			     << joystick.getButtonState(Joystick::L_BUTTON) << " "
			     << joystick.getButtonState(Joystick::R_BUTTON) << endl;
		    */
	
			// erase current footsteps
			while(footstep.steps.size() > 2)
				footstep.steps.pop_back();

			Step step;
			step.stride   = -max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
			step.turn     = -max_turn  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
			step.spacing  = 0.2;
			step.climb    = 0.0;
			step.duration = 0.7;
			footstep.steps.push_back(step);
			footstep.steps.push_back(step);
			footstep.steps.push_back(step);
			step.stride = 0.0;
			step.turn   = 0.0;
			footstep.steps.push_back(step);
		}

		footstep_planner.Plan(param, footstep);
	}

    stepping_controller.Update(timer, param, footstep, centroid, base, foot);
    stabilizer         .Update(timer, param, centroid, base, foot);
	
	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
