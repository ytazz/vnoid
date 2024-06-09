#include "robot.h"
#include "footstep.h"
#include "iksolver.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Robot(){
    base_acc_sensor_name    = "gsensor"  ;
	base_gyro_sensor_name   = "gyrometer";
	right_force_sensor_name = "rfsensor" ;
	left_force_sensor_name  = "lfsensor" ;
}

void Robot::Init(SimpleControllerIO* io, Timer& timer, vector<Joint>& joint){
	io_body = io->body();
	
	if(base_actuation){
        io_body->link(0)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(0));
    }
    for(int i = 0; i < io_body->numLinks(); i++){
	    io->enableInput (io_body->link(i), cnoid::Link::LinkPosition | cnoid::Link::LinkTwist | cnoid::Link::LinkAcceleration);
    }

    joint_pos_filter.resize(joint.size());
	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);
		
        jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity | cnoid::Link::JointAcceleration);
		
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

	timer.count = 0;
	timer.control_count = 0;
	timer.time = 0.0;
	timer.dt = io->timeStep();

}

void Robot::Sense(Timer& timer, Base& base, vector<Joint>& joint){
	// store absolute position and velocity of base link
	{
        Link* lnk = io_body->link(0);
        base.pos = lnk->p();
		base.vel = lnk->v();
		base.acc = lnk->dv();
    }
	if(!base_state_from_simulator){
		if(accel_sensor){
			Vector3 a = accel_sensor->dv();
			base.acc[0] = acc_filter[0](gyro_axis_x.dot(a), timer.dt);
			base.acc[1] = acc_filter[1](gyro_axis_y.dot(a), timer.dt);
			base.acc[2] = acc_filter[2](gyro_axis_z.dot(a), timer.dt);
		}
		if(gyro_sensor){
			Vector3 w = gyro_sensor->w();
			base.angvel[0] = gyro_filter[0](gyro_axis_x.dot(w), timer.dt);
			base.angvel[1] = gyro_filter[1](gyro_axis_y.dot(w), timer.dt);
			base.angvel[2] = gyro_filter[2](gyro_axis_z.dot(w), timer.dt);
		}
	}
	if(base_state_from_simulator){
		base.ori    = Quaternion(io_body->link(0)->R());
		base.angvel = io_body->link(0)->w();
		base.angacc = io_body->link(0)->dw();
		base.angle  = ToRollPitchYaw(base.ori);
	}
	else{
		const double g = 9.8;
		const double angle_correction_gain = 0.01;
		base.angle [0] += (base.angvel[0] + angle_correction_gain*(  base.acc.y() - g*base.angle.x() ))*timer.dt;
		base.angle [1] += (base.angvel[1] + angle_correction_gain*( -base.acc.x() - g*base.angle.y() ))*timer.dt;
		base.angle [2] +=  base.angvel[2]*timer.dt;

		base.ori = FromRollPitchYaw(base.angle);
	}

	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);

        // get position and velocity of each joint
		joint[i].q   = jnt->q ();
		joint[i].dq  = jnt->dq();
		joint[i].ddq = jnt->ddq();
	}
}

void Robot::Sense(Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint){
	Sense(timer, base, joint);

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
