#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
    param.com_height =  0.7;
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

    param.trunk_mass = 24.0;
    param.trunk_com = Vector3(0.0, 0.0, 0.166);

    param.arm_mass[0] = 0.5;
    param.arm_mass[1] = 0.5;
    param.arm_mass[2] = 1.0;
    param.arm_mass[3] = 0.5;
    param.arm_mass[4] = 1.0;
    param.arm_mass[5] = 0.5;
    param.arm_mass[6] = 0.5;
    param.arm_com[0] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[1] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[2] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[3] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[4] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[5] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[6] = Vector3(0.0, 0.0,  0.0);

    param.leg_mass[0] = 0.5;
    param.leg_mass[1] = 0.5;
    param.leg_mass[2] = 1.5;
    param.leg_mass[3] = 1.5;
    param.leg_mass[4] = 0.5;
    param.leg_mass[5] = 0.5;
    param.leg_com[0] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[1] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[2] = Vector3(0.0, 0.0, -0.15);
    param.leg_com[3] = Vector3(0.0, 0.0, -0.20);
    param.leg_com[4] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[5] = Vector3(0.0, 0.0,  0.0);

    param.zmp_min = Vector3(-0.1, -0.05, -0.1);
    param.zmp_max = Vector3( 0.1,  0.05,  0.1);

    param.Init();

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    joint[ 0].Set(1000.0, 200.0, 100.0);
    joint[ 1].Set(1000.0, 200.0, 100.0);
    joint[ 2].Set(1000.0, 200.0, 100.0);
    joint[ 3].Set(1000.0, 200.0, 100.0);
    joint[ 4].Set(1000.0, 200.0, 100.0);
    joint[ 5].Set(1000.0, 200.0, 100.0);
    joint[ 6].Set(1000.0, 200.0, 100.0);
    joint[ 7].Set(1000.0, 200.0, 100.0);
    joint[ 8].Set(1000.0, 200.0, 100.0);
    joint[ 9].Set(1000.0, 200.0, 100.0);
    joint[10].Set(1000.0, 200.0, 100.0);
    joint[11].Set(1000.0, 200.0, 100.0);
    joint[12].Set(1000.0, 200.0, 100.0);
    joint[13].Set(1000.0, 200.0, 100.0);
    joint[14].Set(1000.0, 200.0, 100.0);
    joint[15].Set(1000.0, 200.0, 100.0);
    joint[16].Set(1000.0, 200.0, 100.0);
    joint[17].Set(1000.0, 200.0, 100.0);
    joint[18].Set(1000.0, 200.0, 100.0);
    joint[19].Set(1000.0, 200.0, 100.0);
    joint[20].Set(1000.0, 200.0, 100.0);
    joint[21].Set(1000.0, 200.0, 100.0);
    joint[22].Set(100.0, 20.0, 100.0);
    joint[23].Set(100.0, 20.0, 100.0);
    joint[24].Set(1000.0, 200.0, 100.0);
    joint[25].Set(1000.0, 200.0, 100.0);
    joint[26].Set(1000.0, 200.0, 100.0);
    joint[27].Set(1000.0, 200.0, 100.0);
    joint[28].Set(100.0, 20.0, 100.0);
    joint[29].Set(100.0, 20.0, 100.0);
    
    // needs fast joint movement for step adjustment
    joint_pos_filter_cutoff = 100.0;
    
    // init hardware (simulator interface)
    Robot::Init(io, timer, joint);
    io->enableOutput(io_body->link(0), cnoid::Link::LinkExtWrench);  //< to apply external disturbance

    // set initial state
    centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
    centroid.dcm_ref     = Vector3(0.0, 0.0, param.com_height);
    foot[0].pos_ref = Vector3(0.0, -0.15/2.0, 0.0);
    foot[1].pos_ref = Vector3(0.0,  0.15/2.0, 0.0);

    // init footsteps
    footstep.steps.push_back(Step(0.0, 0.0, 0.15, 0.0, 0.0, 0.5, 0));
    footstep.steps.push_back(Step(0.0, 0.0, 0.15, 0.0, 0.0, 0.5, 1));
    // foot placement of the initial step must be specified
    footstep.steps[0].foot_pos[0] = foot[0].pos_ref;
    footstep.steps[0].foot_pos[1] = foot[1].pos_ref;
    footstep.steps[0].dcm = centroid.dcm_ref;
    footstep_planner.Plan(param, footstep);
    footstep_planner.GenerateDCM(param, footstep);

    footstep_buffer.steps.push_back(footstep.steps[0]);
    footstep_buffer.steps.push_back(footstep.steps[1]);
    
    // init stepping controller
    stepping_controller.swing_height = 0.10;
    stepping_controller.dsp_duration = 0.05;
    
    // init stabilizer
    stabilizer.orientation_ctrl_gain_p = 100.0;
    stabilizer.orientation_ctrl_gain_d = 10.0;
    stabilizer.dcm_ctrl_gain           = 2.0;
        
    InitMarkers(io);

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // comp FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

    while(footstep.steps.size() > 2)
		footstep.steps.pop_back();

    Link* target = io_body->link(0);
    target->f_ext  () = Vector3(0.0, 0.0, 0.0);
	target->tau_ext() = Vector3(0.0, 0.0, 0.0);
    Vector3 offset(0.0, 0.0, 0.0);

    if(4.0 <= timer.time && timer.time <= 4.0 + 0.01){
        Vector3 f(3000.0, 0.0, 0.0);
        target->f_ext  () = f;
	    target->tau_ext() = (target->p() + offset).cross(f);
    }
    /*
    if(5.5 <= timer.time && timer.time <= 5.5 + 0.01){
        Vector3 f(3000.0, 0.0, 0.0);
        target->f_ext  () = f;
	    target->tau_ext() = (target->p() + offset).cross(f);
    }
    if(7.0 <= timer.time && timer.time <= 7.0 + 0.01){
        Vector3 f(3000.0, 0.0, 0.0);
        target->f_ext  () = f;
	    target->tau_ext() = (target->p() + offset).cross(f);
    }
    if(8.0 <= timer.time && timer.time <= 8.0 + 0.01){
        Vector3 f(0.0, 2500.0, 0.0);
        Vector3 offset(0.1, 0.0, -0.1);
        target->f_ext  () = f;
	    target->tau_ext() = (target->p() + offset).cross(f);
    }*/
	Step step;
	step.stride   = 0.1;
	step.turn     = 0.0;
	step.spacing  = 0.2;
	step.climb    = 0.0;
	step.duration = 0.4;
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
		
	footstep_planner.Plan(param, footstep);
    footstep_planner.GenerateDCM(param, footstep);

    // stepping controller generates swing foot trajectory 
    // it also performs landing position adaptation
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot);

    // stabilizer performs balance feedback
    stabilizer.Update(timer, param, centroid, base, foot);

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;
    
    // comp CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

    Robot::Actuate(timer, base, joint);

    UpdateMarkers(base, centroid, hand, foot);
    UpdateMarkers(base, footstep_buffer);
    
	timer.Countup();
}


}
}
