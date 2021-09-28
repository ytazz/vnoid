#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = true;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
	param.com_height =  1.0;
	param.gravity    =  9.8;
    
    // kinematic parameters
    param.base_to_shoulder[0] = Vector3(0.0, -0.1,  0.3);
    param.base_to_shoulder[1] = Vector3(0.0,  0.1,  0.3);
    param.base_to_hip     [0] = Vector3(0.0, -0.1, -0.1);
    param.base_to_hip     [1] = Vector3(0.0,  0.1, -0.1);
    param.wrist_to_hand   [0] = Vector3(0.0,  0.0, -0.1);
    param.wrist_to_hand   [1] = Vector3(0.0,  0.0, -0.1);
    param.ankle_to_foot   [0] = Vector3(0.0,  0.0, -0.05);
    param.ankle_to_foot   [1] = Vector3(0.0,  0.0, -0.05);
    param.arm_joint_index [0] =  4;
    param.arm_joint_index [1] = 11;
    param.leg_joint_index [0] = 18;
    param.leg_joint_index [1] = 24;
    param.upper_arm_length = 0.2;
    param.lower_arm_length = 0.2;
    param.upper_leg_length = 0.3;
    param.lower_leg_length = 0.4;

    param.Init();

    // two hands and twe feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    for(int i = 0; i < 30; i++){
        joint[i].pgain  = 5000.0;
        joint[i].dgain  = 75.0;
        joint[i].ulimit = 100.0;
    }

    // init hardware (simulator interface)
	Robot::Init(io, timer, joint);

    // init footsteps
    footstep.steps.resize(2);

    // init stepping controller
    stepping_controller.Init(param, footstep, centroid, base);

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

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

    //IkSolver::CompLegIk(
	base.pos_ref         = Vector3(0.0, 0.0, 1.5);
    centroid.com_pos_ref = Vector3(0.0, 0.0, 1.5);
    foot[0].pos_ref      = Vector3(0.0, -0.1, 1.5 - 0.7 + 0.1);
    foot[1].pos_ref      = Vector3(0.0,  0.1, 1.5 - 0.7 + 0.2);
    hand[0].pos_ref      = Vector3(0.0, -0.3, 1.5);
    hand[1].pos_ref      = Vector3(0.0,  0.3, 1.5);
    ik_solver.Comp(param, centroid, base, hand, foot, joint);

	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
