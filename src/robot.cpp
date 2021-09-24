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
	nominal_inertia = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(){
	count = 0;
    time  = 0.0;
    dt    = 0.001;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Robot(){
    baseForceSensorName  = "bfsensor" ;
    baseAccSensorName    = "gsensor"  ;
	baseGyroSensorName   = "gyrometer";
	rightForceSensorName = "rfsensor" ;
	leftForceSensorName  = "lfsensor" ;
    gyroAxisX = Vector3(1.0, 0.0, 0.0);
    gyroAxisY = Vector3(0.0, 1.0, 0.0);
    gyroAxisZ = Vector3(0.0, 0.0, 1.0);
    
	joystickCycle = 10;

    baseActuation = false;
}

void Robot::Init(SimpleControllerIO* io, Timer* timer, vector<Joint>& joint){
	ioBody = io->body();
	
	// set actuation mode of root link as link position
	//  this is needed to retrieve root link pose as input
	//ioBody->link(0)->setActuationMode(cnoid::Link::LINK_POSITION);
	//io->enableIO(ioBody->link(0));

    if(baseActuation){
        ioBody->link(0)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(ioBody->link(0));
    }
	io->enableInput(ioBody->link(0), cnoid::Link::LinkPosition);

	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = ioBody->joint(i);
		//jnt->setActuationMode(cnoid::Link::JOINT_VELOCITY);

		// set torque actuation mode
		//  this sets joint angle as input by default
		//  joint velocity needs to be set as input explicitly
		jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity);
		
		joint[i].q_ref  = joint[i].q  = jnt->q ();
		joint[i].dq_ref = joint[i].dq = jnt->dq();
		joint[i].u      = 0.0;
	}
		
	{
        forceSensor = ioBody->findDevice<ForceSensor       >(baseForceSensorName);
		accelSensor = ioBody->findDevice<AccelerationSensor>(baseAccSensorName  );
		gyroSensor  = ioBody->findDevice<RateGyroSensor    >(baseGyroSensorName );
        if(forceSensor) io->enableInput(forceSensor);
        if(accelSensor) io->enableInput(accelSensor);
        if(gyroSensor ) io->enableInput(gyroSensor );
	}

	for(int i = 0; i < 2; i++){
		footForceSensor[i] = ioBody->findDevice<ForceSensor>(i == 0 ? rightForceSensorName : leftForceSensorName);
        if(footForceSensor[i])
            io->enableInput(footForceSensor[i]);
	}
	
	timer->dt = io->timeStep();

}

void Robot::Sense(Base* base, Foot* foot){
	if(count % joystickCycle == 0){
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
	}
	// angular momentum feedback
	// gyro of RHP model is rotated
    if(gyroSensor){
        base->angvel[0] = gyroAxisX.dot(gyroSensor->w());
        base->angvel[1] = gyroAxisY.dot(gyroSensor->w());
        base->angvel[2] = gyroAxisZ.dot(gyroSensor->w());
    }
	base->angle [0] += base->angvel[0]*dt;
	base->angle [1] += base->angvel[1]*dt;

	for(int i = 0; i < 2; i++){
		// get force/moment from force sensor
        if(footForceSensor[i]){
		    foot[i].force  = footForceSensor[i]->F().segment<3>(0);
		    foot[i].moment = footForceSensor[i]->F().segment<3>(3);
        }
    }

}

void Robot::Actuate(Base* base, vector<Joint>& joint){
    if(baseActuation){
        Link* lnk = ioBody->link(0);
        lnk->p() = base->pos_ref;
        lnk->R() = base->ori_ref.matrix();
    }
    for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = ioBody->joint(i);
		
		joint[i].q  = jnt->q ();
		joint[i].dq = jnt->dq();
		joint[i].CalcTorque(dt);
		jnt->u() = joint[i].u;
	}
}

void Robot::Countup(){
	count++;
	time += dt;
}

}
}
