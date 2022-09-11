#include "robot.h"
#include "footstep.h"
#include "iksolver.h"

namespace cnoid{
namespace vnoid{

///////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(){
	pgain  = 0.0;
	dgain  = 0.0;
	ulimit = 0.0;

	q      = 0.0;
	dq     = 0.0;
	q_ref  = 0.0;
	dq_ref = 0.0;
	u      = 0.0;
}

void Joint::Set(double _pgain, double _dgain, double _ulimit){
    pgain  = _pgain;
    dgain  = _dgain;
    ulimit = _ulimit;
}

void Joint::CalcTorque(){
	u = pgain*(q_ref - q) + dgain*(dq_ref - dq);
	u = std::min(std::max(-ulimit, u), ulimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Centroid::Centroid(){
	force_ref   = Vector3(0.0, 0.0, 0.0);
	moment_ref  = Vector3(0.0, 0.0, 0.0);
	zmp         = Vector3(0.0, 0.0, 0.0);
	zmp_ref     = Vector3(0.0, 0.0, 0.0);
	com_pos_ref = Vector3(0.0, 0.0, 0.0);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	com_acc_ref = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Base::Base(){
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	angle      = Vector3(0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	angvel     = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Hand::Hand(){
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);
	arm_twist  = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Foot::Foot(){
    contact          = false;
	contact_ref      = false;
	balance          = 0.0;
	balance_ref      = 0.0;
	pos_ref          = Vector3(0.0, 0.0, 0.0);
	ori_ref          = Quaternion(1.0, 0.0, 0.0, 0.0);
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

Param::Param(){
    total_mass       = 50.0;
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

    Init();
}

void Param::Init(){
    T = sqrt(com_height/gravity);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(){
	count = 0;
    time  = 0.0;
    dt    = 0.001;
}

void Timer::Countup(){
	count++;
	time += dt;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Robot(){
    base_acc_sensor_name    = "gsensor"  ;
	base_gyro_sensor_name   = "gyrometer";
	right_force_sensor_name = "rfsensor" ;
	left_force_sensor_name  = "lfsensor" ;

    gyro_axis_x = Vector3(1.0, 0.0, 0.0);
    gyro_axis_y = Vector3(0.0, 1.0, 0.0);
    gyro_axis_z = Vector3(0.0, 0.0, 1.0);
    
	base_actuation            = false;
    gyro_filter_cutoff        = 20.0;
    acc_filter_cutoff         = 20.0;
    foot_force_filter_cutoff  = 20.0;
    foot_moment_filter_cutoff = 20.0;
    joint_pos_filter_cutoff   = 10.0;
	
}

void Robot::Init(SimpleControllerIO* io, Timer& timer, vector<Joint>& joint){
	io_body = io->body();
	
	if(base_actuation){
        io_body->link(0)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(0));
    }
	io->enableInput(io_body->link(0), cnoid::Link::LinkPosition);

    joint_pos_filter.resize(joint.size());
	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);
		
        jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity);
		
		joint[i].q_ref  = joint[i].q  = jnt->q ();
		joint[i].dq_ref = joint[i].dq = jnt->dq();
		joint[i].u      = 0.0;

        joint_pos_filter[i].SetCutoff(joint_pos_filter_cutoff);
	}
		
	{
        accel_sensor = io_body->findDevice<AccelerationSensor>(base_acc_sensor_name  );
		gyro_sensor  = io_body->findDevice<RateGyroSensor    >(base_gyro_sensor_name );

        if(accel_sensor) io->enableInput(accel_sensor);
        if(gyro_sensor ) io->enableInput(gyro_sensor );

        for(int j = 0; j < 3; j++){
            acc_filter [j].SetCutoff(acc_filter_cutoff );
            gyro_filter[j].SetCutoff(gyro_filter_cutoff);
        }
	}

	for(int i = 0; i < 2; i++){
		foot_force_sensor[i] = io_body->findDevice<ForceSensor>(i == 0 ? right_force_sensor_name : left_force_sensor_name);
        if(foot_force_sensor[i])
            io->enableInput(foot_force_sensor[i]);

        for(int j = 0; j < 3; j++){
            foot_force_filter [i][j].SetCutoff(foot_force_filter_cutoff );
            foot_moment_filter[i][j].SetCutoff(foot_moment_filter_cutoff);
        }
	}
	
	timer.dt = io->timeStep();

}

void Robot::Sense(Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint){
	if(gyro_sensor){
        Vector3 w = gyro_sensor->w();
        base.angvel[0] = gyro_filter[0](gyro_axis_x.dot(w), timer.dt);
        base.angvel[1] = gyro_filter[1](gyro_axis_y.dot(w), timer.dt);
        base.angvel[2] = gyro_filter[2](gyro_axis_z.dot(w), timer.dt);
    }
	base.angle [0] += base.angvel[0]*timer.dt;
	base.angle [1] += base.angvel[1]*timer.dt;

	for(int i = 0; i < 2; i++){
		// get force/moment from force sensor
        if(foot_force_sensor[i]){
            Vector3 f = foot_force_sensor[i]->F().segment<3>(0);
            Vector3 m = foot_force_sensor[i]->F().segment<3>(3);
            for(int j = 0; j < 3; j++){
		        foot[i].force [j] = foot_force_filter [i][j](f[j], timer.dt);
		        foot[i].moment[j] = foot_moment_filter[i][j](m[j], timer.dt);
            }
        }
    }

    for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);

        // get position and velocity of each joint
		joint[i].q  = jnt->q ();
		joint[i].dq = jnt->dq();
	}
}

void Robot::Actuate(Timer& timer, Base& base, vector<Joint>& joint){
    // if base actuation is enabled, directly specify the position and orientation of the base link
    if(base_actuation){
        Link* lnk = io_body->link(0);
        lnk->p() = base.pos_ref;
        lnk->R() = base.ori_ref.matrix();
    }

    for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);

        // filter position reference
        joint[i].q_ref  = joint_pos_filter[i](joint[i].q_ref, timer.dt);
        // velocity reference if taken from the time derivative of the filter output
        joint[i].dq_ref = joint_pos_filter[i].yd;
		
		// determine joint torque by PD control
        joint[i].CalcTorque();
		jnt->u() = joint[i].u;
	}
}

}
}
