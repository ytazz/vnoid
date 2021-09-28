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

void Joint::CalcTorque(double dt){
	u = pgain*(q_ref - q) + dgain*(dq_ref - dq);
	u = std::min(std::max(-ulimit, u), ulimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Centroid::Centroid(){
	force_ref   = Vector3(0.0, 0.0, 0.0);
	moment_ref  = Vector3(0.0, 0.0, 0.0);
	moment_mod  = Vector3(0.0, 0.0, 0.0);
	zmp         = Vector3(0.0, 0.0, 0.0);
	zmp_ref     = Vector3(0.0, 0.0, 0.0);
	zmp_mod     = Vector3(0.0, 0.0, 0.0);
	com_pos_ref = Vector3(0.0, 0.0, 0.0);
	com_pos_cor = Vector3(0.0, 0.0, 0.0);
	com_pos_mod = Vector3(0.0, 0.0, 0.0);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	com_vel_cor = Vector3(0.0, 0.0, 0.0);
	com_vel_mod = Vector3(0.0, 0.0, 0.0);
	com_acc_ref = Vector3(0.0, 0.0, 0.0);
	com_acc_cor = Vector3(0.0, 0.0, 0.0);
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
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Foot::Foot(){
    contact          = false;
	contact_ref      = false;
	contact_duration = 0.0;
	balance          = 0.0;
	balance_ref      = 0.0;
	pos_ref          = Vector3(0.0, 0.0, 0.0);
	pos_mod          = Vector3(0.0, 0.0, 0.0);
	ori_ref          = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle_ref        = Vector3(0.0, 0.0, 0.0);
	angle_mod        = Vector3(0.0, 0.0, 0.0);
	vel_ref          = Vector3(0.0, 0.0, 0.0);
	vel_mod          = Vector3(0.0, 0.0, 0.0);
	angvel_ref       = Vector3(0.0, 0.0, 0.0);
	angvel_mod       = Vector3(0.0, 0.0, 0.0);
	acc_ref          = Vector3(0.0, 0.0, 0.0);
	angacc_ref       = Vector3(0.0, 0.0, 0.0);
	force            = Vector3(0.0, 0.0, 0.0);
	force_ref        = Vector3(0.0, 0.0, 0.0);
	force_error      = Vector3(0.0, 0.0, 0.0);
	moment           = Vector3(0.0, 0.0, 0.0);
	moment_ref       = Vector3(0.0, 0.0, 0.0);
	moment_error     = Vector3(0.0, 0.0, 0.0);
	zmp              = Vector3(0.0, 0.0, 0.0);
	zmp_ref          = Vector3(0.0, 0.0, 0.0);
	dpos             = Vector3(0.0, 0.0, 0.0);
	drot             = Vector3(0.0, 0.0, 0.0);
	tliftoff         = 0.0;
	tlanding         = 0.0;
	foothold         = Vector3(0.0, 0.0, 0.0);
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
    base_force_sensor_name  = "bfsensor" ;
    base_acc_sensor_name    = "gsensor"  ;
	base_gyro_sensor_name   = "gyrometer";
	right_force_sensor_name = "rfsensor" ;
	left_force_sensor_name  = "lfsensor" ;

    gyro_axis_x = Vector3(1.0, 0.0, 0.0);
    gyro_axis_y = Vector3(0.0, 1.0, 0.0);
    gyro_axis_z = Vector3(0.0, 0.0, 1.0);
    
	base_actuation = false;
}

void Robot::Init(SimpleControllerIO* io, Timer& timer, vector<Joint>& joint){
	io_body = io->body();
	
	if(base_actuation){
        io_body->link(0)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(0));
    }
	io->enableInput(io_body->link(0), cnoid::Link::LinkPosition);

	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);
		
        jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity);
		
		joint[i].q_ref  = joint[i].q  = jnt->q ();
		joint[i].dq_ref = joint[i].dq = jnt->dq();
		joint[i].u      = 0.0;
	}
		
	{
        force_sensor = io_body->findDevice<ForceSensor       >(base_force_sensor_name);
		accel_sensor = io_body->findDevice<AccelerationSensor>(base_acc_sensor_name  );
		gyro_sensor  = io_body->findDevice<RateGyroSensor    >(base_gyro_sensor_name );

        if(force_sensor) io->enableInput(force_sensor);
        if(accel_sensor) io->enableInput(accel_sensor);
        if(gyro_sensor ) io->enableInput(gyro_sensor );
	}

	for(int i = 0; i < 2; i++){
		foot_force_sensor[i] = io_body->findDevice<ForceSensor>(i == 0 ? right_force_sensor_name : left_force_sensor_name);
        if(foot_force_sensor[i])
            io->enableInput(foot_force_sensor[i]);
	}
	
	timer.dt = io->timeStep();

}

void Robot::Sense(Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint){
	// angular momentum feedback
	// gyro of RHP model is rotated
    if(gyro_sensor){
        base.angvel[0] = gyro_axis_x.dot(gyro_sensor->w());
        base.angvel[1] = gyro_axis_y.dot(gyro_sensor->w());
        base.angvel[2] = gyro_axis_z.dot(gyro_sensor->w());
    }
	base.angle [0] += base.angvel[0]*timer.dt;
	base.angle [1] += base.angvel[1]*timer.dt;

	for(int i = 0; i < 2; i++){
		// get force/moment from force sensor
        if(foot_force_sensor[i]){
		    foot[i].force  = foot_force_sensor[i]->F().segment<3>(0);
		    foot[i].moment = foot_force_sensor[i]->F().segment<3>(3);
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
		
		// determine joint torque by PD control
        joint[i].CalcTorque(timer.dt);
		jnt->u() = joint[i].u;
	}
}

}
}
