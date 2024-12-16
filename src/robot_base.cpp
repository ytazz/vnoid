#include "robot_base.h"
#include "footstep.h"
#include "iksolver.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

///////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(){
	pgain  = 0.0;
	dgain  = 0.0;
	ulimit = 0.0;

	q      = 0.0;
	dq     = 0.0;
	ddq    = 0.0;
    q_ref  = 0.0;
	dq_ref = 0.0;
	u      = 0.0;
	u_ref  = 0.0;
}

void Joint::Set(double _pgain, double _dgain, double _ulimit){
    pgain  = _pgain;
    dgain  = _dgain;
    ulimit = _ulimit;
}

void Joint::CalcTorque(){
	u = u_ref + pgain*(q_ref - q) + dgain*(dq_ref - dq);
	u = std::min(std::max(-ulimit, u), ulimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Centroid::Centroid(){
	force_ref   = Vector3(0.0, 0.0, 0.0);
	moment_ref  = Vector3(0.0, 0.0, 0.0);
	zmp         = Vector3(0.0, 0.0, 0.0);
	zmp_ref     = Vector3(0.0, 0.0, 0.0);
	zmp_target  = Vector3(0.0, 0.0, 0.0);
	dcm         = Vector3(0.0, 0.0, 0.0);
	dcm_ref     = Vector3(0.0, 0.0, 0.0);
	dcm_target  = Vector3(0.0, 0.0, 0.0);
	com_pos     = Vector3(0.0, 0.0, 0.0);
	com_pos_ref = Vector3(0.0, 0.0, 0.0);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	com_acc_ref = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Base::Base(){
	pos        = Vector3(0.0, 0.0, 0.0);
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	angle      = Vector3(0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	ori        = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	angvel     = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	acc        = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	angacc     = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Hand::Hand(){
	pos        = Vector3(0.0, 0.0, 0.0);
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	ori        = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle      = Vector3(0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);
	arm_twist  = 0.0;

	fix_arm_twist  = true;
    fix_elbow_dir  = false;
    arm_twist      = 0.0;
	elbow_dir      = Vector3::Zero();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Foot::Foot(){
    contact          = false;
	contact_ref      = false;
	balance          = 0.0;
	balance_ref      = 0.0;
	pos              = Vector3(0.0, 0.0, 0.0);
	pos_ref          = Vector3(0.0, 0.0, 0.0);
	ori              = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref          = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle            = Vector3(0.0, 0.0, 0.0);
	angle_ref        = Vector3(0.0, 0.0, 0.0);
	vel_ref          = Vector3(0.0, 0.0, 0.0);
	angvel_ref       = Vector3(0.0, 0.0, 0.0);
	acc_ref          = Vector3(0.0, 0.0, 0.0);
	angacc_ref       = Vector3(0.0, 0.0, 0.0);
	force            = Vector3(0.0, 0.0, 0.0);
	force_ref        = Vector3(0.0, 0.0, 0.0);
	moment           = Vector3(0.0, 0.0, 0.0);
	moment_ref       = Vector3(0.0, 0.0, 0.0);
	zmp              = Vector3(0.0, 0.0, 0.0);
	zmp_ref          = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Ground::Ground(){
	angle = Vector3(0.0, 0.0, 0.0);
	ori   = Quaternion(1.0, 0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Param::Param(){
    total_mass       = 50.0;
	nominal_inertia  = Vector3(20.0, 20.0, 5.0);
	com_height       = 1.0;
	gravity          = 9.8;
    
    base_to_shoulder[0] = Vector3(0.0, 0.0, 0.0);
    base_to_shoulder[1] = Vector3(0.0, 0.0, 0.0);
    base_to_hip     [0] = Vector3(0.0, 0.0, 0.0);
    base_to_hip     [1] = Vector3(0.0, 0.0, 0.0);
    arm_joint_index [0] = 0;
    arm_joint_index [1] = 0;
    leg_joint_index [0] = 0;
    leg_joint_index [1] = 0;
    
    upper_arm_length = 0.2;
    lower_arm_length = 0.2;
    upper_leg_length = 0.3;
    lower_leg_length = 0.4;

	trunk_mass = 1.0;
	trunk_com  = Vector3(0.0, 0.0, 0.0);
	
	for (int i = 0; i < 7; i++) {
        arm_mass[i] = 0.0;
        arm_com[i] = Vector3(0.0, 0.0, 0.0);
    }
    for (int i = 0; i < 6; i++) {
        leg_mass[i] = 0.0;
        leg_com[i] = Vector3(0.0, 0.0, 0.0);
    }

    Init();
}

void Param::Init(){
    T = sqrt(com_height/gravity);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(){
	count = 0;
    control_count = 0;
    time  = 0.0;
    dt    = 0.001;
}

void Timer::Countup(){
	count++;
	time += dt;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

RobotBase::RobotBase(){
    gyro_axis_x = Vector3(1.0, 0.0, 0.0);
    gyro_axis_y = Vector3(0.0, 1.0, 0.0);
    gyro_axis_z = Vector3(0.0, 0.0, 1.0);
    
	base_state_from_simulator = false;
	base_actuation            = false;
    gyro_filter_cutoff        = 20.0;
    acc_filter_cutoff         = 20.0;
    foot_force_filter_cutoff  = 20.0;
    foot_moment_filter_cutoff = 20.0;
    joint_pos_filter_cutoff   = 10.0;
	
}

RobotBase::~RobotBase(){

}

}
}
