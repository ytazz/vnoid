#include "stepping_controller.h"

#include "robot.h"
#include "footstep.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

SteppingController::SteppingController(){
    swing_height        = 0.05;
    swing_tilt          = 0.0;
    dsp_duration        = 0.1;
    descend_duration    = 0.0;
    descend_depth       = 0.0;
    landing_adjust_rate = 0.0;
}

void SteppingController::Init(const Param& param, Centroid& centroid, Base& base){
	tswitch = 0.0;
	zdiff   = 0.0;

	base.ori_ref    = Quaternion();
	base.angvel_ref = Vector3(0.0, 0.0, 0.0);

	centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
	centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
	centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
	centroid.dcm_ref     = Vector3(0.0, 0.0, 0.0);
	centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
}

bool SteppingController::CheckLanding(const Timer& timer, Footstep& footstep, vector<Foot>& foot){
	// step duration elapsed
    double t = timer.time;
    if(t >= tswitch + footstep.steps[0].duration){
        return true;
    }
    /*
    // force on landing foot exceeded threshold
    if( t >= tswitch + 0.9*footstep.steps[0].duration && foot[!footstep.steps[0].side].contact ){
        return true;
    }
    */
    return false;
}

void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot){
    if(CheckLanding(timer, footstep, foot)){
        // store z difference between footstep and foot ref
        int swg = !footstep.steps[0].side;
        //zdiff = foot[swg].pos_ref.z() - footstep.steps[1].foot_pos[swg].z();
        zdiff = 0.0;
        
        // store switching time
        tswitch = timer.time;

        // reset timing adjust flag
        adjusted = false;

        if(footstep.steps.size() > 1){
            // pop step just completed from the footsteps
            footstep.steps.pop_front();
            if(footstep.steps.size() == 1){
		        printf("end of footstep reached\n");
	        }
        }
	}

    if(footstep.steps.size() < 2){
		return;
	}

    Step& st0 = footstep.steps[0];
    Step& st1 = footstep.steps[1];
    int sup =  st0.side;
	int swg = !st0.side;

    // time elapsed since last switch
    double t = timer.time - tswitch;

	// calc reference dcm
    centroid.dcm_ref = centroid.com_pos_ref + param.T*centroid.com_vel_ref;
    
    // calc desired zmp that bring the dcm to the reference dcm at next landing
	if(st0.duration - t > 0.001){
		double alpha = exp((st0.duration - t)/param.T);
		centroid.zmp_ref.x() = (st1.dcm.x() - alpha*centroid.dcm_ref.x())/(1.0 - alpha);
		centroid.zmp_ref.y() = (st1.dcm.y() - alpha*centroid.dcm_ref.y())/(1.0 - alpha);
        centroid.zmp_ref.z() = (st1.dcm.z() - alpha*(centroid.dcm_ref.z() - (param.com_height + zdiff)))/(1.0 - alpha);
	}
    
    // update reference com pos, vel, acc
    centroid.com_pos_ref += centroid.com_vel_ref * timer.dt;
    //centroid.com_pos_ref.z() = param.com_height + zdiff;
	centroid.com_vel_ref += centroid.com_acc_ref * timer.dt;
	centroid.com_acc_ref  = Vector3(
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[0] - centroid.zmp_ref[0]),
		(1.0/(param.T*param.T))*(centroid.com_pos_ref[1] - centroid.zmp_ref[1]),
        (1.0/(param.T*param.T))*(centroid.com_pos_ref[2] - (centroid.zmp_ref[2] + param.com_height + zdiff))
        );

    // reference base orientation is set as the middle of feet orientation
    double angle_diff = foot[1].angle_ref.z() - foot[0].angle_ref.z();
    if(angle_diff >  pi) angle_diff -= 2.0*pi;
    if(angle_diff < -pi) angle_diff += 2.0*pi;
	base.angle_ref.z() = foot[0].angle_ref.z() + angle_diff/2.0;

    base.ori_ref   = FromRollPitchYaw(base.angle_ref);

    // set support foot position
    foot[sup].pos_ref     = st0.foot_pos  [sup] + Vector3(0.0, 0.0, zdiff);
    foot[sup].angle_ref   = st0.foot_angle[sup];
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

	// set swing foot position
    //  if lift-off and landing poses are the same or it is double support phase
    const double eps_pos   = 0.005;
    const double eps_angle = 0.1*(3.14/180.0);
    if( ( (st0.foot_pos  [swg] - st1.foot_pos  [swg]).norm() < eps_pos &&
          (st0.foot_angle[swg] - st1.foot_angle[swg]).norm() < eps_angle ) ||
         t < dsp_duration )
    {
        foot[swg].pos_ref     = st0.foot_pos  [swg] + Vector3(0.0, 0.0, zdiff);
        foot[swg].angle_ref   = st0.foot_angle[swg];
        foot[swg].contact_ref = true;
    }
    else{
        double ts   = t - dsp_duration;            //< time elapsed in ssp
        double tauv = st0.duration - dsp_duration; //< duration of vertical movement
        double tauh = tauv - descend_duration;     //< duration of horizontal movement

        // cycloid swing profile
        double sv     = ts/tauv;
        double sh     = ts/tauh;
        double thetav = 2.0*pi*sv;
        double thetah = 2.0*pi*sh;
        double ch     = (sh < 1.0 ? (thetah - sin(thetah))/(2.0*pi) : 1.0);
        double cv     = (1.0 - cos(thetav))/2.0;
        double cv2    = (1.0 - cos(thetav/2.0))/2.0;
        double cw     = sin(thetah);

        // landing position
        Vector3 pos_land = st1.foot_pos[swg];
        // adjust landing position
        Vector3 w = base.ori_ref*base.angvel;
        Vector3 pdiff(
             landing_adjust_rate*w.y(),
            -landing_adjust_rate*w.x(),
             0.0);
        pos_land += pdiff;

        // foot turning
        Vector3 turn = st1.foot_angle[swg] - st0.foot_angle[swg];
        if(turn.z() >  pi) turn.z() -= 2.0*pi;
        if(turn.z() < -pi) turn.z() += 2.0*pi;

        // foot tilting
        Vector3 tilt = st0.foot_ori[swg]*Vector3(0.0, swing_tilt, 0.0);

        foot[swg].pos_ref      = (1.0 - ch)*st0.foot_pos[swg] + ch*pos_land;
        foot[swg].pos_ref.z() += (cv*(swing_height + 0.5*descend_depth) - cv2*descend_depth + zdiff);
        foot[swg].angle_ref    = st0.foot_angle[swg] + ch*turn + cw*tilt;
        foot[swg].contact_ref  = false;
    }
    foot[swg].ori_ref     = FromRollPitchYaw(foot[swg].angle_ref);
	
}

}
}
