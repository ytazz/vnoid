#include "stepping_controller.h"

namespace cnoid{
namespace vnoid{

SteppingController::SteppingController(){
}

void SteppingController::Init(const Robot& robot, const Footstep& footstep){
	cur_step[0] = footstep[0];
	cur_step[1] = footstep[1];
	step_count  = 0;
	tswitch     = 0.0;
	
	base_ori    = 0.0;
	base_angvel = 0.0;
	com_pos_ref = Vector3(0.0, 0.0, robot.com_height);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	com_acc_ref = Vector3(0.0, 0.0, 0.0);
	dcm_ref     = Vector3(0.0, 0.0, 0.0);
	dcm_diff    = Vector3(0.0, 0.0, 0.0);
	zmp_ref     = Vector3(0.0, 0.0, 0.0);
	zmp_diff    = Vector3(0.0, 0.0, 0.0);
}

void SteppingController::FromRobot(){
	// do nothing during standby
	if(robot->time < standby_period)
		return;

	double t = std::max(0.0, robot->time - standby_period);

	if(t >= tswitch + cur_step[0].duration && steps.size() > 1){
		step_count++;
		steps.pop_front();
		
		if(steps.size() == 1){
			printf("end of footstep reached\n");
			return;
		}
	
		cur_step[0].side = !cur_step[0].side;

		// determine next landing position
		Step& st0 = steps[0];
		Step& st1 = steps[1];
		int sup =  cur_step[0].side;
		int swg = !cur_step[0].side;

		cur_step[1].side = !cur_step[0].side;

		// support foot does not move
		cur_step[1].foot_pos[sup] = cur_step[0].foot_pos[sup];
		cur_step[1].foot_ori[sup] = cur_step[0].foot_ori[sup];
			
		Eigen::AngleAxisd R(cur_step[1].foot_ori[sup] - st1.foot_ori[sup], Vector3::UnitZ());
		cur_step[1].foot_pos[swg] = R*(st1.foot_pos[swg] - st1.foot_pos[sup]) + cur_step[1].foot_pos[sup];
		cur_step[1].foot_ori[swg] =   (st1.foot_ori[swg] - st1.foot_ori[sup]) + cur_step[1].foot_ori[sup];
		cur_step[1].dcm           = R*(st1.dcm           - st1.foot_pos[sup]) + cur_step[1].foot_pos[sup];
		cur_step[1].zmp           = R*(st1.zmp           - st1.foot_pos[sup]) + cur_step[1].foot_pos[sup];
		
		cur_step[0].duration = st0.duration;

		tswitch = t;
	}
}

void SteppingController::ToRobot(){
	double t = std::max(0.0, robot->time - standby_period);

	int sup =  cur_step[0].side;
	int swg = !cur_step[0].side;

	if(robot->time > standby_period){
		if(steps.size() == 1){

		}
		else{
			// update swing foot position
			Vector2 pf, vf;
			double  rf, wf;
			double  vz;
			swing->GetTraj(t - tswitch, pf, rf, vf, wf);
			swing->GetVerticalVelocity(pf, vf, cur_step[0].foot_pos[swg][2], vz);

			cur_step[0].foot_pos   [swg][0]  = pf.x();
			cur_step[0].foot_pos   [swg][1]  = pf.y();
			cur_step[0].foot_pos   [swg][2] += vz*robot->dt;
			cur_step[0].foot_ori   [swg]     = rf;
			cur_step[0].foot_vel   [swg][0]  = vf.x();
			cur_step[0].foot_vel   [swg][1]  = vf.y();
			cur_step[0].foot_vel   [swg][2]  = vz;
			cur_step[0].foot_angvel[swg]     = wf;

			// calc zmp 
			if(cur_step[0].duration - (t - tswitch) > 0.001){
				double alpha = exp((cur_step[0].duration - (t - tswitch))/robot->T);
				zmp_ref = (cur_step[1].dcm - alpha*dcm_ref)/(1.0 - alpha);

				// K=2 just flips the sign of dcm dynamics (thus make it stable)
				zmp_diff = 2.0*dcm_diff;
			}

			// update references
    		com_pos_ref += com_vel_ref * robot->dt;
			com_vel_ref += com_acc_ref * robot->dt;
			com_acc_ref  = Vector3(
				(1.0/(robot->T*robot->T))*(com_pos_ref[0] - zmp_ref[0]),
				(1.0/(robot->T*robot->T))*(com_pos_ref[1] - zmp_ref[1]),
				0.0);

			base_ori    = (cur_step[0].foot_ori[sup] + cur_step[0].foot_ori[swg])/2.0;
			base_angvel =  cur_step[0].foot_angvel[swg]/2.0;

			dcm_ref  = com_pos_ref + robot->T*com_vel_ref;
		}
	}

	const double eps = 1.0e-5;

	Vector3 angle;
	angle = Vector3(0.0, 0.0, cur_step[0].foot_ori[sup]);
	robot->foot[sup].pos_ref     = cur_step[0].foot_pos[sup];
	robot->foot[sup].angle_ref   = angle;
	robot->foot[sup].ori_ref     = FromRollPitchYaw(angle);
	robot->foot[sup].vel_ref     = Vector3(0.0, 0.0, 0.0);
	robot->foot[sup].angvel_ref  = Vector3(0.0, 0.0, 0.0);
	robot->foot[sup].contact_ref = true;

	angle = Vector3(0.0, 0.0, cur_step[0].foot_ori[swg]);
	robot->foot[swg].pos_ref     = cur_step[0].foot_pos[swg];
	robot->foot[swg].angle_ref   = angle;
	robot->foot[swg].ori_ref     = FromRollPitchYaw(angle);
	robot->foot[swg].vel_ref     = cur_step[0].foot_vel[swg];
	robot->foot[swg].angvel_ref  = Vector3(0.0, 0.0, cur_step[0].foot_angvel[swg]);
	robot->foot[swg].contact_ref = cur_step[0].foot_pos[swg][2] < eps; //(t - tswitch > swing->duration - swing->dsp_duration);
	
	angle = Vector3(0.0, 0.0, base_ori);
	robot->base.angle_ref  = angle;
	robot->base.ori_ref    = FromRollPitchYaw(angle);
	robot->base.angvel_ref = Vector3(0.0, 0.0, base_angvel);
	
	robot->centroid.com_pos_ref  = com_pos_ref;
	robot->centroid.com_vel_ref  = com_vel_ref;
	robot->centroid.zmp_ref      = zmp_ref /*+ zmpDiff*/;
}

}
}
