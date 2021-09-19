#include "robot.h"
#include "footstep.h"
#include "iksolver.h"

namespace cnoid{
namespace vnoid{

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Joint::Joint(){
	pgain  = 0.0;
	dgain  = 0.0;
	ulimit = 0.0;

	q      = 0.0;
	dq     = 0.0;
	q_ref  = 0.0;
	q_des  = 0.0;
	dq_ref = 0.0;
	u_ref  = 0.0;
	u      = 0.0;
}

void Robot::Joint::CalcTorque(double dt){
	u = 0.0*u_ref + 1.0*(pgain*(q_ref - q) + dgain*(dq_ref - dq));
	u = std::min(std::max(-ulimit, u), ulimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Centroid::Centroid(){
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Base::Base(){
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Foot::Foot(){

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
    
	min_contact_force        = 1.0;
	force_ctrl_damping       = 0.0;
	force_ctrl_gain          = 0.0;
	force_ctrl_limit         = 0.0;
	moment_ctrl_damping      = 0.0;
	moment_ctrl_gain         = 0.0;
	moment_ctrl_limit        = 0.0;
	orientation_ctrl_gain_p  = 0.0;
	orientation_ctrl_gain_d  = 0.0;
	swing_height_adjust_rate = 0.0;

	nominal_inertia = Vector3(0.0, 0.0, 0.0);
	com_pos_gain = 0.0;
	com_vel_gain = 0.0;

	downsample = 1;

	joystickCycle = 10;

    baseActuation = false;

    footstep = 0;
    iksolver = 0;
}

void Robot::Init(SimpleControllerIO* io){
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
        base.forceSensor = ioBody->findDevice<ForceSensor       >(baseForceSensorName);
		base.accelSensor = ioBody->findDevice<AccelerationSensor>(baseAccSensorName  );
		base.gyroSensor  = ioBody->findDevice<RateGyroSensor    >(baseGyroSensorName );
        if(base.forceSensor) io->enableInput(base.forceSensor);
        if(base.accelSensor) io->enableInput(base.accelSensor);
        if(base.gyroSensor ) io->enableInput(base.gyroSensor );

		base.pos_ref    = Vector3(0.0, 0.0, 0.0);
		base.angle      = Vector3(0.0, 0.0, 0.0);
		base.angle_ref  = Vector3(0.0, 0.0, 0.0);
		base.ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
		base.vel_ref    = Vector3(0.0, 0.0, 0.0);
		base.angvel     = Vector3(0.0, 0.0, 0.0);
		base.angvel_ref = Vector3(0.0, 0.0, 0.0);
		base.acc_ref    = Vector3(0.0, 0.0, 0.0);
		base.angacc_ref = Vector3(0.0, 0.0, 0.0);
	}
	{
		centroid.force_ref   = Vector3(0.0, 0.0, 0.0);
		centroid.moment_ref  = Vector3(0.0, 0.0, 0.0);
		centroid.moment_mod  = Vector3(0.0, 0.0, 0.0);
		centroid.cop         = Vector3(0.0, 0.0, 0.0);
		centroid.cop_ref     = Vector3(0.0, 0.0, 0.0);
		centroid.cop_mod     = Vector3(0.0, 0.0, 0.0);
		centroid.com_pos_ref = Vector3(0.0, 0.0, 0.0);
		centroid.com_pos_cor = Vector3(0.0, 0.0, 0.0);
		centroid.com_pos_mod = Vector3(0.0, 0.0, 0.0);
		centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
		centroid.com_vel_cor = Vector3(0.0, 0.0, 0.0);
		centroid.com_vel_mod = Vector3(0.0, 0.0, 0.0);
		centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
		centroid.com_acc_cor = Vector3(0.0, 0.0, 0.0);
	}
	for(int i = 0; i < 2; i++){
		hand[i].pos_ref    = Vector3(0.0, 0.0, 0.0);
		hand[i].vel_ref    = Vector3(0.0, 0.0, 0.0);
		hand[i].acc_ref    = Vector3(0.0, 0.0, 0.0);
		hand[i].ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
		hand[i].angle_ref  = Vector3(0.0, 0.0, 0.0);
		hand[i].angvel_ref = Vector3(0.0, 0.0, 0.0);
		hand[i].angacc_ref = Vector3(0.0, 0.0, 0.0);
	}
	for(int i = 0; i < 2; i++){
		foot[i].forceSensor = ioBody->findDevice<ForceSensor>(i == 0 ? rightForceSensorName : leftForceSensorName);
        if(foot[i].forceSensor) io->enableInput(foot[i].forceSensor);

		foot[i].contact          = false;
		foot[i].contact_ref      = false;
		foot[i].contact_duration = 0.0;
		foot[i].balance          = 0.0;
		foot[i].balance_ref      = 0.0;
		foot[i].pos_ref          = Vector3(0.0, 0.0, 0.0);
		foot[i].pos_mod          = Vector3(0.0, 0.0, 0.0);
		foot[i].ori_ref          = Quaternion(1.0, 0.0, 0.0, 0.0);
		foot[i].angle_ref        = Vector3(0.0, 0.0, 0.0);
		foot[i].angle_mod        = Vector3(0.0, 0.0, 0.0);
		foot[i].vel_ref          = Vector3(0.0, 0.0, 0.0);
		foot[i].vel_mod          = Vector3(0.0, 0.0, 0.0);
		foot[i].angvel_ref       = Vector3(0.0, 0.0, 0.0);
		foot[i].angvel_mod       = Vector3(0.0, 0.0, 0.0);
		foot[i].acc_ref          = Vector3(0.0, 0.0, 0.0);
		foot[i].angacc_ref       = Vector3(0.0, 0.0, 0.0);
		foot[i].force            = Vector3(0.0, 0.0, 0.0);
		foot[i].force_ref        = Vector3(0.0, 0.0, 0.0);
		foot[i].force_error      = Vector3(0.0, 0.0, 0.0);
		foot[i].moment           = Vector3(0.0, 0.0, 0.0);
		foot[i].moment_ref       = Vector3(0.0, 0.0, 0.0);
		foot[i].moment_error     = Vector3(0.0, 0.0, 0.0);
		foot[i].cop              = Vector3(0.0, 0.0, 0.0);
		foot[i].cop_ref          = Vector3(0.0, 0.0, 0.0);
		foot[i].dpos             = Vector3(0.0, 0.0, 0.0);
		foot[i].drot             = Vector3(0.0, 0.0, 0.0);
		foot[i].tliftoff         = 0.0;
		foot[i].tlanding         = 0.0;
		foot[i].foothold         = Vector3(0.0, 0.0, 0.0);
	}
	
	dt    = io->timeStep();
	count = 0;
	time  = 0.0;

	logFile = fopen(logFilename.c_str(), "w");
    if(logFile){
	    fprintf(logFile,
		    "time "
		    "base_angle_x base_angle_y base_angle_z "
		    "base_angvel_x base_angvel_y base_angvel_z "
		    "com_pos_ref_x com_pos_ref_y com_pos_ref_z "
		    "com_pos_mod_x com_pos_mod_y com_pos_mod_z "
		    "com_vel_ref_x com_vel_ref_y com_vel_ref_z "
		    "com_vel_mod_x com_vel_mod_y com_vel_mod_z "
		    "cop_pos_ref_x cop_pos_ref_y cop_pos_ref_z "
		    "mom_x mom_y mom_z "
		    "foot0_px foot0_py foot0_pz foot0_ax foot0_ay foot0_az "
		    "foot0_vx foot0_vy foot0_vz foot0_wx foot0_wy foot0_wz "
		    "foot0_fx foot0_fy foot0_fz foot0_mx foot0_my foot0_mz "
		    "foot1_px foot1_py foot1_pz foot1_ax foot1_ay foot1_az "
		    "foot1_vx foot1_vy foot1_vz foot1_wx foot1_wy foot1_wz "
		    "foot1_fx foot1_fy foot1_fz foot1_mx foot1_my foot1_mz "
		    "\n"
	    );
    }
}

void Robot::CompFCop(){
    // get actual force from the sensor
	for(Foot& ft : foot){
		// get force/moment from force sensor
        if(ft.forceSensor){
		    ft.force  = ft.forceSensor->F().segment<3>(0);
		    ft.moment = ft.forceSensor->F().segment<3>(3);
        }
	
		// set contact state
		ft.contact = (ft.force[2] >= min_contact_force);

		// measure continuous contact duration
		if(ft.contact){
			ft.contact_duration += dt;
			ft.cop = Vector3(-ft.moment[1]/ft.force[2], ft.moment[0]/ft.force[2], 0.0);
		}
		else{
			ft.contact_duration = 0.0;
			ft.cop = Vector3(0.0, 0.0, 0.0);
		}
	}

	// both feet not in contact
	if(!foot[0].contact && !foot[1].contact){
		foot[0].balance = 0.5;
		foot[1].balance = 0.5;
		centroid.cop = Vector3(0.0, 0.0, 0.0);
	}
	else{
		double f0 = std::max(0.0, foot[0].force[2]);
		double f1 = std::max(0.0, foot[1].force[2]);
		foot[0].balance = f0/(f0 + f1);
		foot[1].balance = f1/(f0 + f1);
		centroid.cop =
			     (foot[0].balance) * (foot[0].pos_ref + foot[0].ori_ref * foot[0].cop)
	           + (foot[1].balance) * (foot[1].pos_ref + foot[1].ori_ref * foot[1].cop);
	}
}

void Robot::CompICop(){
	// switch based on contact state
	if(!foot[0].contact_ref && !foot[1].contact_ref){
		foot[0].balance_ref = 0.5;
		foot[1].balance_ref = 0.5;
		foot[0].cop_ref = Vector3(0.0, 0.0, 0.0);
		foot[1].cop_ref = Vector3(0.0, 0.0, 0.0);
	}
	if( foot[0].contact_ref && !foot[1].contact_ref){
		foot[0].balance_ref = 1.0;
		foot[1].balance_ref = 0.0;
		foot[0].cop_ref = foot[0].ori_ref.conjugate() * ((centroid.cop_ref + centroid.cop_mod) - foot[0].pos_ref);
		foot[1].cop_ref = Vector3(0.0, 0.0, 0.0);
	}
	if(!foot[0].contact_ref &&  foot[1].contact_ref){
		foot[0].balance_ref = 0.0;
		foot[1].balance_ref = 1.0;
		foot[0].cop_ref = Vector3(0.0, 0.0, 0.0);
		foot[1].cop_ref = foot[1].ori_ref.conjugate() * ((centroid.cop_ref + centroid.cop_mod) - foot[1].pos_ref);
	}
	if( foot[0].contact_ref &&  foot[1].contact_ref){
		//
		Vector2 b;
		Vector3 pdiff  = foot[1].pos_ref - foot[0].pos_ref;
		double  pdiff2 = pdiff.squaredNorm();
		const double eps = 1.0e-10;
		if(pdiff2 < eps){
			b[0] = b[1] = 0.5;
		}
		else{
			b[0] = (pdiff.dot(foot[1].pos_ref - (centroid.cop_ref + centroid.cop_mod)))/pdiff2;
			b[0] = std::min(std::max(0.0, b[0]), 1.0);
			b[1] = 1.0 - b[0];
		}

		foot[0].balance_ref = b[0];
		foot[1].balance_ref = b[1];

		Vector3 cop_proj = b[0]*foot[0].pos_ref + b[1]*foot[1].pos_ref;

		double b2 = b.squaredNorm();
		foot[0].cop_ref = (b[0]/b2) * (foot[0].ori_ref.conjugate() * ((centroid.cop_ref + centroid.cop_mod) - cop_proj));
		foot[1].cop_ref = (b[1]/b2) * (foot[1].ori_ref.conjugate() * ((centroid.cop_ref + centroid.cop_mod) - cop_proj));
	}

	for(int i = 0; i < 2; i++){
		// force and moment to realize desired CoP
		foot[i].force_ref     =  foot[i].ori_ref.conjugate() * (foot[i].balance_ref * centroid.force_ref);
		foot[i].moment_ref[0] =  foot[i].force_ref[2] * foot[i].cop_ref[1];
		foot[i].moment_ref[1] = -foot[i].force_ref[2] * foot[i].cop_ref[0];
		foot[i].moment_ref[2] =  foot[i].balance_ref * centroid.moment_ref[2];

		//DSTR << foot[i].force_ref[2] << " " << foot[i].cop_ref[0] << " " << foot[i].cop_ref[1] << endl;
	}

}

void Robot::Sense(){
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
    if(base.gyroSensor){
        base.angvel[0] = gyroAxisX.dot(base.gyroSensor->w());
        base.angvel[1] = gyroAxisY.dot(base.gyroSensor->w());
        base.angvel[2] = gyroAxisZ.dot(base.gyroSensor->w());
	    //base.angvel[0] = -base.waistGyro->w()[2];
	    //base.angvel[1] =  base.waistGyro->w()[0];
    }
	base.angle [0] += base.angvel[0]*dt;
	base.angle [1] += base.angvel[1]*dt;

	// calculate cop from measured forces
	CompFCop();
}

void Robot::Control(){
	// copy reference values from iksolver
	centroid.force_ref[0] = total_mass* centroid.com_acc_ref[0];
	centroid.force_ref[1] = total_mass* centroid.com_acc_ref[1];
	centroid.force_ref[2] = total_mass*(centroid.com_acc_ref[2] + gravity);
	centroid.moment_ref   = Vector3(0.0, 0.0, 0.0);

	// feedback base orientation to cop
	{
		Vector3 m(0.0, 0.0, 0.0);
		m[0] = orientation_ctrl_gain_p * (base.angle_ref[0] - base.angle[0]) + orientation_ctrl_gain_d * (base.angvel_ref[0] - base.angvel[0]);
		m[1] = orientation_ctrl_gain_p * (base.angle_ref[1] - base.angle[1]) + orientation_ctrl_gain_d * (base.angvel_ref[1] - base.angvel[1]);

		centroid.moment_mod = base.ori_ref * m;
		centroid.cop_mod    = base.ori_ref * ((1.0/centroid.force_ref[2])*Vector3(-m[1], m[0], 0.0));
	}

	// calculate desired forces from desired cop
	CompICop();

	for(int i = 0; i < 2; i++){
		foot[i].pos_mod    = Vector3(0.0, 0.0, 0.0);
		foot[i].angle_mod  = Vector3(0.0, 0.0, 0.0);
		foot[i].vel_mod    = Vector3(0.0, 0.0, 0.0);
		foot[i].angvel_mod = Vector3(0.0, 0.0, 0.0);
	}

	for(int i = 0; i < 2; i++){
		// ground reaction force control
		if( foot[i].contact ){
			foot[i].force_error[2] = foot[i].force_ref[2] - foot[i].force[2];
			foot[i].dpos[2] += (-force_ctrl_damping*foot[i].dpos[2] + force_ctrl_gain*foot[i].force_error[2])*dt;
			foot[i].dpos[2] = std::min(std::max(-force_ctrl_limit, foot[i].dpos[2]), force_ctrl_limit);

			foot[i].moment_error[0] = foot[i].moment_ref[0] - foot[i].moment[0];
			foot[i].moment_error[1] = foot[i].moment_ref[1] - foot[i].moment[1];
			foot[i].drot[0] += (-moment_ctrl_damping*foot[i].drot[0] + moment_ctrl_gain*foot[i].moment_error[0])*dt;
			foot[i].drot[1] += (-moment_ctrl_damping*foot[i].drot[1] + moment_ctrl_gain*foot[i].moment_error[1])*dt;
			foot[i].drot[0] = std::min(std::max(-moment_ctrl_limit, foot[i].drot[0]), moment_ctrl_limit);
			foot[i].drot[1] = std::min(std::max(-moment_ctrl_limit, foot[i].drot[1]), moment_ctrl_limit);

			// feedback to desired foot pose
			foot[i].pos_mod   += -foot[i].dpos;
			foot[i].angle_mod += -foot[i].drot;
			//foot[i].vel_mod    += -foot[i].dpos;
			//foot[i].angvel_mod += -foot[i].drot;
		}
	}

	// swing foot height compensation
	if( ( foot[0].contact_ref && !foot[1].contact_ref) ||
	    (!foot[0].contact_ref &&  foot[1].contact_ref) ){
		int sup = (foot[0].contact_ref ? 0 : 1);
		int swg = !sup;
		Vector3 diff = base.ori_ref.conjugate()*(foot[swg].pos_ref - foot[sup].pos_ref);
		foot[swg].pos_mod[2] += swing_height_adjust_rate*(diff[0]*base.angle[1] - diff[1]*base.angle[0]);
	}

			
	// comp IK
    if(iksolver)
    	iksolver->Comp(this, dt);

    if(baseActuation){
        Link* lnk = ioBody->link(0);
        lnk->p() = base.pos_ref;
        lnk->R() = base.ori_ref.matrix();
    }
    for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = ioBody->joint(i);
		
		joint[i].q  = jnt->q ();
		joint[i].dq = jnt->dq();
		joint[i].CalcTorque(dt);
		jnt->u() = joint[i].u;
	}
    
    // update footstep markers
    if(footstep)
        footstep->UpdateMarkers();
}

void Robot::Countup(){
	count++;
	time += dt;
}

void Robot::Save(){
    if(logFile){
	    fprintf(logFile, "%f", time);
	    fprintf(logFile, " %f %f %f",
		    base.angle(0),
		    base.angle(1),
		    base.angle(2));
	    fprintf(logFile, " %f %f %f",
		    base.angvel(0),
		    base.angvel(1),
		    base.angvel(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.com_pos_ref(0),
		    centroid.com_pos_ref(1),
		    centroid.com_pos_ref(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.com_pos_mod(0),
		    centroid.com_pos_mod(1),
		    centroid.com_pos_mod(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.com_vel_ref(0),
		    centroid.com_vel_ref(1),
		    centroid.com_vel_ref(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.com_vel_mod(0),
		    centroid.com_vel_mod(1),
		    centroid.com_vel_mod(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.cop_ref(0),
		    centroid.cop_ref(1),
		    centroid.cop_ref(2));
	    fprintf(logFile, " %f %f %f",
		    centroid.moment_ref(0),
		    centroid.moment_ref(1),
		    centroid.moment_ref(2));
	    for(int i = 0; i < 2; i++){
		    fprintf(logFile, " %f %f %f %f %f %f",
			    foot[i].pos_ref   (0)/* - centroid.com_pos_ref(0)*/,
			    foot[i].pos_ref   (1)/* - centroid.com_pos_ref(1)*/,
			    foot[i].pos_ref   (2)/* - centroid.com_pos_ref(2)*/,
			    foot[i].angle_ref (0),
			    foot[i].angle_ref (1),
			    foot[i].angle_ref (2));
		    fprintf(logFile, " %f %f %f %f %f %f",
			    foot[i].vel_ref   (0),
			    foot[i].vel_ref   (1),
			    foot[i].vel_ref   (2),
			    foot[i].angvel_ref(0),
			    foot[i].angvel_ref(1),
			    foot[i].angvel_ref(2));
		    fprintf(logFile, " %f %f %f %f %f %f",
			    foot[i].force_ref (0),
			    foot[i].force_ref (1),
			    foot[i].force_ref (2),
			    foot[i].moment_ref(0),
			    foot[i].moment_ref(1),
			    foot[i].moment_ref(2));
	    }
	    fprintf(logFile, "\n");
	    fflush(logFile);
    }
}

}
}
