#include "stepping_controller.h"

#include "robot.h"
#include "footstep.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

SteppingController::SteppingController(){
    swing_height = 0.05;
}

void SteppingController::Init(const Param& param, const Footstep& footstep, Centroid& centroid, Base& base){
	cur_step[0] = footstep.steps[0];
	cur_step[1] = footstep.steps[1];
	
    step_index  = 0;
	tswitch     = 0.0;
	
	base.ori_ref    = Quaternion();
	base.angvel_ref = Vector3(0.0, 0.0, 0.0);

	centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
	centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
	centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
	centroid.dcm_ref     = Vector3(0.0, 0.0, 0.0);
	centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
}


void SteppingController::CalcSwing(double t, Vector3& pos, double& ori, Vector3& vel, double& angvel) {
	/*
    // no swing movement if foothold does not change
	if(p_swg == p_land && r_swg == r_land){
		p = p_swg;
		r = r_swg;
		v = vec2_t();
		w = 0.0;
		return;
	}
	// after landing
	real_t tau_travel = duration - dsp_duration;
	if(t > tau_travel){
		p = p_land;
		v = vec2_t();
		r = r_land;
		w = 0.0;
		return;
	}

	if(type == Type::Spline){
		p = InterpolatePos(t,
			0.0       , p_swg , v_swg,
			tau_travel, p_land, vec2_t(),
			Interpolate::Cubic);
		v = InterpolateVel(t,
			0.0       , p_swg , v_swg,
			tau_travel, p_land, vec2_t(),
			Interpolate::Cubic);

		r = InterpolatePos(t,
			0.0       , r_swg , w_swg,
			tau_travel, r_land, 0.0,
			Interpolate::Cubic);
		w = InterpolateVel(t,
			0.0       , r_swg , w_swg,
			tau_travel, r_land, 0.0,
			Interpolate::Cubic);

	}
    */
}

void SteppingController::Update(const Timer& timer, const Param& param, const Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot){
	double t = timer.time;

	if(t >= tswitch + cur_step[0].duration && footstep.steps.size() > 1){
		step_index++;
		
        if(footstep.steps.size() < step_index + 2){
			printf("end of footstep reached\n");
			return;
		}
	
		cur_step[0].side = !cur_step[0].side;

		// determine next landing position
		const Step& st0 = footstep.steps[step_index+0];
		const Step& st1 = footstep.steps[step_index+1];
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

    int sup =  cur_step[0].side;
	int swg = !cur_step[0].side;

    // set support foot position
    foot[sup].pos_ref     = cur_step[0].foot_pos[sup];
    foot[sup].angle_ref   = Vector3(0.0, 0.0, cur_step[0].foot_ori[sup]);
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

	// set swing foot position
    //  if lift-off and landing poses are the same
    if( cur_step[0].foot_pos[swg] == cur_step[1].foot_pos[swg] &&
        cur_step[0].foot_ori[swg] == cur_step[1].foot_ori[swg] ){
        foot[swg].pos_ref       = cur_step[0].foot_pos[swg];
        foot[swg].angle_ref.z() = cur_step[0].foot_ori[swg];
    }
    else{
        // cycloid swing profile
        double s     = (t - tswitch)/cur_step[0].duration;
        double theta = 2.0*pi*s;
        double ch    = (theta - sin(theta))/(2.0*pi);
        double cv    = (1.0 - cos(theta))/2.0;

        // foot turning
        double turn = cur_step[1].foot_ori[swg] - cur_step[0].foot_ori[swg];
        if(turn >  pi) turn -= 2.0*pi;
        if(turn < -pi) turn += 2.0*pi;

        foot[swg].pos_ref      = (1.0 - ch)*cur_step[0].foot_pos[swg] + ch*cur_step[1].foot_pos[swg];
        foot[swg].pos_ref.z() += cv*swing_height;
        foot[swg].angle_ref    = Vector3(0.0, 0.0, cur_step[0].foot_ori[swg] + ch*turn);
    }
    foot[swg].ori_ref     = FromRollPitchYaw(foot[swg].angle_ref);
    foot[swg].contact_ref = false;

	// calc reference dcm
    centroid.dcm_ref = centroid.com_pos_ref + param.T*centroid.com_vel_ref;

    // calc reference zmp 
	if(cur_step[0].duration - (t - tswitch) > 0.001){
		double alpha = exp((cur_step[0].duration - (t - tswitch))/param.T);
		centroid.zmp_ref = (cur_step[1].dcm - alpha*centroid.dcm_ref)/(1.0 - alpha);
	}

    // update reference com pos, vel, acc
    centroid.com_pos_ref += centroid.com_vel_ref * timer.dt;
	centroid.com_vel_ref += centroid.com_acc_ref * timer.dt;
	centroid.com_acc_ref  = Vector3(
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[0] - centroid.zmp_ref[0]),
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[1] - centroid.zmp_ref[1]),
		0.0);

    // calc reference base orientation
	base.angle_ref = Vector3(0.0, 0.0, (cur_step[0].foot_ori[sup] + cur_step[0].foot_ori[swg])/2.0);
    base.ori_ref   = FromRollPitchYaw(base.angle_ref);
	
}

}
}
