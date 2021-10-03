#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = false;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
	param.com_height =  0.8;
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

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    joint[ 0].Set( 5000.0,  75.0, 1000.0);
    joint[ 1].Set( 5000.0,  75.0, 1000.0);
    joint[ 2].Set( 5000.0,  75.0, 1000.0);
    joint[ 3].Set( 5000.0,  75.0, 1000.0);
    joint[ 4].Set( 5000.0,  75.0, 1000.0);
    joint[ 5].Set( 5000.0,  75.0, 1000.0);
    joint[ 6].Set( 5000.0,  75.0, 1000.0);
    joint[ 7].Set( 5000.0,  75.0, 1000.0);
    joint[ 8].Set( 5000.0,  75.0, 1000.0);
    joint[ 9].Set( 5000.0,  75.0, 1000.0);
    joint[10].Set( 5000.0,  75.0, 1000.0);
    joint[11].Set( 5000.0,  75.0, 1000.0);
    joint[12].Set( 5000.0,  75.0, 1000.0);
    joint[13].Set( 5000.0,  75.0, 1000.0);
    joint[14].Set( 5000.0,  75.0, 1000.0);
    joint[15].Set( 5000.0,  75.0, 1000.0);
    joint[16].Set( 5000.0,  75.0, 1000.0);
    joint[17].Set( 5000.0,  75.0, 1000.0);
    joint[18].Set(20000.0, 300.0, 1000.0);
    joint[19].Set(20000.0, 300.0, 1000.0);
    joint[20].Set(20000.0, 300.0, 1000.0);
    joint[21].Set(20000.0, 300.0, 1000.0);
    joint[22].Set( 5000.0,  75.0, 1000.0);
    joint[23].Set( 5000.0,  75.0, 1000.0);
    joint[24].Set(20000.0, 300.0, 1000.0);
    joint[25].Set(20000.0, 300.0, 1000.0);
    joint[26].Set(20000.0, 300.0, 1000.0);
    joint[27].Set(20000.0, 300.0, 1000.0);
    joint[28].Set( 5000.0,  75.0, 1000.0);
    joint[29].Set( 5000.0,  75.0, 1000.0);
    
    // init hardware (simulator interface)
	Robot::Init(io, timer, joint);

    // init footsteps
    footstep.steps.push_back(Step(0.0, 0.0, 0.15, 0.0, 0.0, 0.5, 0));
    footstep.steps.push_back(Step(0.0, 0.0, 0.15, 0.0, 0.0, 0.5, 1));
    // foot placement of the initial step must be specified
    footstep.steps[0].foot_pos[0] = Vector3(0.0, -0.15/2.0, 0.0);
    footstep.steps[0].foot_pos[1] = Vector3(0.0,  0.15/2.0, 0.0);
    footstep_planner.Plan(param, footstep);

    // init stepping controller
    stepping_controller.dsp_duration = 0.15;
    stepping_controller.Init(param, centroid, base);

    // init stabilizer
    stabilizer.swing_height_adjust_rate = 0.0;
    stabilizer.orientation_ctrl_gain_p  = 1000.0;
    stabilizer.orientation_ctrl_gain_d  = 200.0;
    stabilizer.min_contact_force        = 1.0;
    stabilizer.force_ctrl_damping       = 5.0;
    stabilizer.force_ctrl_gain          = 0.0001;
    stabilizer.force_ctrl_limit         = 0.01;
    stabilizer.moment_ctrl_damping      = 5.0;
    stabilizer.moment_ctrl_gain         = 0.0001;
    stabilizer.moment_ctrl_limit        = 0.5;

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

	if(timer.count % 10 == 0){
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
		step.stride   = 0.2; //-max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
		step.turn     = 0.1; //-max_turn  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
		step.spacing  = 0.15;
		step.climb    = 0.0;
		step.duration = 0.5;
		footstep.steps.push_back(step);
		footstep.steps.push_back(step);
		footstep.steps.push_back(step);
		step.stride = 0.0;
		step.turn   = 0.0;
		footstep.steps.push_back(step);
		
		footstep_planner.Plan(param, footstep);
	}

    stepping_controller.Update(timer, param, footstep, centroid, base, foot);
    stabilizer         .Update(timer, param, centroid, base, foot);

    /*
    // IK testing
    joystick.readCurrentState();
    double jv = joystick.getPosition(Joystick::L_STICK_V_AXIS);
    double jh = joystick.getPosition(Joystick::L_STICK_H_AXIS);

    centroid.com_pos_ref = Vector3(0.0, 0.0, 1.5);
    foot[0].pos_ref      = Vector3(0.2*jh, -0.1, 1.5 - 0.7 + 0.1*jv);
    foot[1].pos_ref      = Vector3(-0.2*jh,  0.1, 1.5 - 0.7 - 0.1*jv);
    hand[0].pos_ref      = Vector3(0.1, -0.3, 1.5 + 0.1*jh);
    hand[1].pos_ref      = Vector3(-0.1,  0.3, 1.5 + 0.1*jh);
    */

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;
    ik_solver.Comp(param, centroid, base, hand, foot, joint);

	base.pos_ref         = Vector3(0.0, 0.0, 1.5);

    Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
