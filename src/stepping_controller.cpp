#include "stepping_controller.h"

#include "robot.h"
#include "footstep.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

SteppingController::SteppingController(){
    swing_height = 0.05;
    dsp_duration = 0.1;
}

void SteppingController::Init(const Param& param, Centroid& centroid, Base& base){
	tswitch     = 0.0;
	
	base.ori_ref    = Quaternion();
	base.angvel_ref = Vector3(0.0, 0.0, 0.0);

	centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
	centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
	centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
	centroid.dcm_ref     = Vector3(0.0, 0.0, 0.0);
	centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
}

void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot){
	double t = timer.time;

    // step duration elapsed
	if(t >= tswitch + footstep.steps[0].duration && footstep.steps.size() > 1){
        // pop step just completed from the footsteps
        footstep.steps.pop_front();
        if(footstep.steps.size() == 1){
		    printf("end of footstep reached\n");
	    }

        tswitch = t;
	}

    if(footstep.steps.size() < 2){
		return;
	}

    Step& st0 = footstep.steps[0];
    Step& st1 = footstep.steps[1];
    int sup =  st0.side;
	int swg = !st0.side;

    // set support foot position
    foot[sup].pos_ref     = st0.foot_pos[sup];
    foot[sup].angle_ref   = Vector3(0.0, 0.0, st0.foot_ori[sup]);
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

	// set swing foot position
    //  if lift-off and landing poses are the same or it is double support phase
    if( (st0.foot_pos[swg] == st1.foot_pos[swg] &&
         st0.foot_ori[swg] == st1.foot_ori[swg] ) ||
         t - tswitch < dsp_duration )
    {
        foot[swg].pos_ref     = st0.foot_pos[swg];
        foot[swg].angle_ref   = Vector3(0.0, 0.0, st0.foot_ori[swg]);
        foot[swg].contact_ref = true;
    }
    else{
        // cycloid swing profile
        double s     = (t - tswitch - dsp_duration)/(st0.duration - dsp_duration);
        double theta = 2.0*pi*s;
        double ch    = (theta - sin(theta))/(2.0*pi);
        double cv    = (1.0 - cos(theta))/2.0;

        // foot turning
        double turn = st1.foot_ori[swg] - st0.foot_ori[swg];
        if(turn >  pi) turn -= 2.0*pi;
        if(turn < -pi) turn += 2.0*pi;

        foot[swg].pos_ref      = (1.0 - ch)*st0.foot_pos[swg] + ch*st1.foot_pos[swg];
        foot[swg].pos_ref.z() += cv*swing_height;
        foot[swg].angle_ref    = Vector3(0.0, 0.0, st0.foot_ori[swg] + ch*turn);
        foot[swg].contact_ref  = false;
    }
    foot[swg].ori_ref     = FromRollPitchYaw(foot[swg].angle_ref);

	// calc reference dcm
    centroid.dcm_ref = centroid.com_pos_ref + param.T*centroid.com_vel_ref;

    // calc reference zmp 
	if(st0.duration - (t - tswitch) > 0.001){
		double alpha = exp((st0.duration - (t - tswitch))/param.T);
		centroid.zmp_ref = (st1.dcm - alpha*centroid.dcm_ref)/(1.0 - alpha);
	}

    // update reference com pos, vel, acc
    centroid.com_pos_ref += centroid.com_vel_ref * timer.dt;
	centroid.com_vel_ref += centroid.com_acc_ref * timer.dt;
	centroid.com_acc_ref  = Vector3(
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[0] - centroid.zmp_ref[0]),
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[1] - centroid.zmp_ref[1]),
		0.0);

    // reference base orientation is set as the middle of feet orientation
    double angle_diff = foot[1].angle_ref.z() - foot[0].angle_ref.z();
    if(angle_diff >  pi) angle_diff -= 2.0*pi;
    if(angle_diff < -pi) angle_diff += 2.0*pi;
	base.angle_ref.z() = foot[0].angle_ref.z() + angle_diff/2.0;
    base.ori_ref   = FromRollPitchYaw(base.angle_ref);
	
}

}
}
