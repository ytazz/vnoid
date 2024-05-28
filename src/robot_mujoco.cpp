#include "robot_mujoco.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

RobotMujoco::RobotMujoco(){

}

void RobotMujoco::Init(mjModel* _m, mjData* _d, const Param& param, Timer& timer, vector<Joint>& joint){
    m   = _m;
    d   = _d;
    
    joint_pos_filter.resize(joint.size());
	for (int i = 0; i < joint.size(); ++i) {
		joint[i].q_ref  = joint[i].q  = d->qpos[7+i];
		joint[i].dq_ref = joint[i].dq = d->qvel[6+i];
		joint[i].u      = 0.0;

        joint_pos_filter[i].SetCutoff(joint_pos_filter_cutoff);
	}
		
	{
        for(int j = 0; j < 3; j++){
            acc_filter [j].SetCutoff(acc_filter_cutoff );
            gyro_filter[j].SetCutoff(gyro_filter_cutoff);
        }
	}

	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 3; j++){
            foot_force_filter [i][j].SetCutoff(foot_force_filter_cutoff );
            foot_moment_filter[i][j].SetCutoff(foot_moment_filter_cutoff);
        }
	}
	
	timer.dt = param.control_cycle*m->opt.timestep;

}

void RobotMujoco::Sense(Timer& timer, Base& base, vector<Joint>& joint){
	// store absolute position and velocity of base link
	{
        base.pos = Vector3(d->qpos[0], d->qpos[1], d->qpos[2]);
		base.vel = Vector3(d->qvel[0], d->qvel[1], d->qvel[2]);
    }

    // first 7 elements of sensordata are framepos and framequat
    int idx = 7;
    Vector3 a(d->sensordata[idx+0], d->sensordata[idx+1], d->sensordata[idx+2]); idx += 3;
	base.acc[0] = acc_filter[0](gyro_axis_x.dot(a), timer.dt);
	base.acc[1] = acc_filter[1](gyro_axis_y.dot(a), timer.dt);
	base.acc[2] = acc_filter[2](gyro_axis_z.dot(a), timer.dt);

    Vector3 w(d->sensordata[idx+0], d->sensordata[idx+1], d->sensordata[idx+2]); idx += 3;
    base.angvel[0] = gyro_filter[0](gyro_axis_x.dot(w), timer.dt);
    base.angvel[1] = gyro_filter[1](gyro_axis_y.dot(w), timer.dt);
    base.angvel[2] = gyro_filter[2](gyro_axis_z.dot(w), timer.dt);

	if(base_state_from_simulator){
		base.ori    = Quaternion(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
		base.angvel = Vector3(d->qvel[3], d->qvel[4], d->qvel[5]);
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
		// get position and velocity of each joint
		joint[i].q  = d->qpos[7+i];
		joint[i].dq = d->qvel[6+i];
	}
}

void RobotMujoco::Sense(Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint){
	Sense(timer, base, joint);

    // first 13 elements of sensordata are framepos, framequat, accel, and gyro
    int idx = 13;
	for(int i = 0; i < 2; i++){
		// get force/moment from force sensor
        Vector3 f(d->sensordata[idx+0], d->sensordata[idx+1], d->sensordata[idx+2]); idx += 3;
        Vector3 m(d->sensordata[idx+0], d->sensordata[idx+1], d->sensordata[idx+2]); idx += 3;
        for(int j = 0; j < 3; j++){
		    foot[i].force [j] = foot_force_filter [i][j](f[j], timer.dt);
		    foot[i].moment[j] = foot_moment_filter[i][j](m[j], timer.dt);
        }
    }
}

void RobotMujoco::Actuate(Timer& timer, Base& base, vector<Joint>& joint){
    if(base_actuation){
        // base actuation not implemented
        // any way to do it using mujoco actuator?
    }

    for (int i = 0; i < joint.size(); ++i) {
        // filter position reference
        joint[i].q_ref  = joint_pos_filter[i](joint[i].q_ref, timer.dt);
        // velocity reference if taken from the time derivative of the filter output
        joint[i].dq_ref = joint_pos_filter[i].yd;

		// determine joint torque by PD control
        joint[i].CalcTorque();
		d->ctrl[i] = joint[i].u;
	}
}

}
}
