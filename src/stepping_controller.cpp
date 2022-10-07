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

    buffer_ready = false;
}

bool SteppingController::CheckLanding(const Timer& timer, Step& step, vector<Foot>& foot){
	// step duration elapsed
    double t = timer.time;
    if(t >= step.tbegin + step.duration){
        return true;
    }
    
    // force on landing foot exceeded threshold
    //if( t >= step.tbegin + step.duration - 0.1 && foot[!step.side].contact ){
    //    return true;
    //}
    
    return false;
}

void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot){
    if(CheckLanding(timer, footstep_buffer.steps[0], foot)){
        if(footstep.steps.size() > 1){
            // pop step just completed from the footsteps
            footstep.steps.pop_front();
            if(footstep.steps.size() == 1){
		        printf("end of footstep reached\n");
                return;
	        }
        }

        footstep_buffer.steps[1].dcm = footstep_buffer.steps[0].dcm;
        footstep_buffer.steps.pop_front();
        footstep_buffer.steps.push_back(Step());
        footstep_buffer.steps[0].tbegin = timer.time;

        buffer_ready = false;
    }

    Step& st0  = footstep.steps[0];
    Step& st1  = footstep.steps[1];
    Step& stb0 = footstep_buffer.steps[0];
    Step& stb1 = footstep_buffer.steps[1];
    int sup =  st0.side;
    int swg = !st0.side;
	
    double T  = param.T;
    Vector3 offset = Vector3(0.0, 0.0, param.com_height);
    
    if(!buffer_ready){
        // update support, lift-off, and landing positions
        stb0.side = st0.side;
        stb1.side = st1.side;

        stb0.stepping = st0.stepping;
        stb0.duration = st0.duration;

        stb0.foot_pos  [sup] = foot[sup].pos_ref;
        stb0.foot_angle[sup] = Vector3(0.0, 0.0, foot[sup].angle_ref.z());
        stb0.foot_ori  [sup] = FromRollPitchYaw(stb0.foot_angle[sup]);
        stb0.foot_pos  [swg] = foot[swg].pos_ref;
        stb0.foot_angle[swg] = Vector3(0.0, 0.0, foot[swg].angle_ref.z());
        stb0.foot_ori  [swg] = FromRollPitchYaw(stb0.foot_angle[swg]);
    
        // landing position relative to support foot, taken from footsteps
        Quaternion ori_rel = st0.foot_ori[sup].conjugate()* st1.foot_ori[swg];
        Vector3    pos_rel = st0.foot_ori[sup].conjugate()*(st1.foot_pos[swg] - st0.foot_pos[sup]);
        Vector3    dcm_rel = st0.foot_ori[sup].conjugate()*(st1.dcm - st0.foot_pos[sup]);
    
        // calc absolute landing position
        stb1.foot_pos  [sup] = stb0.foot_pos  [sup];
        stb1.foot_ori  [sup] = stb0.foot_ori  [sup];
        stb1.foot_angle[sup] = stb0.foot_angle[sup];
        stb1.foot_pos  [swg] = stb0.foot_pos[sup] + stb0.foot_ori[sup]*pos_rel;
        stb1.foot_ori  [swg] = stb0.foot_ori[sup]*ori_rel;
        stb1.foot_angle[swg] = ToRollPitchYaw(stb1.foot_ori[swg]);
        stb1.dcm = stb0.foot_pos[sup] + stb0.foot_ori[sup]*dcm_rel;

        // calc zmp
        double alpha = exp(-stb0.duration/T);
        stb0.zmp = (1.0/(1.0 - alpha))*(stb0.dcm - alpha*stb1.dcm) - offset;

        buffer_ready = true;
    }

    if(footstep.steps.size() < 2){
		return;
	}

    // time to landing
    double ttl = stb0.tbegin + stb0.duration - timer.time;
	double alpha = exp(-ttl/T);
    
	stb0.dcm = (1.0-alpha)*(stb0.zmp + offset) + alpha*stb1.dcm;
    
    if(timer.time - stb0.tbegin > dsp_duration){
        // landing adjustment
        Vector3 land_rel = (st1.foot_pos[swg] - st0.foot_pos[sup]) - (stb0.dcm - stb0.foot_pos[sup]);
	    stb1.foot_pos[swg].x() = centroid.dcm_ref.x() + land_rel.x();
	    stb1.foot_pos[swg].y() = centroid.dcm_ref.y() + land_rel.y();
    }
    
    // reference base orientation is set as the middle of feet orientation
    double angle_diff = foot[1].angle_ref.z() - foot[0].angle_ref.z();
    while(angle_diff >  pi) angle_diff -= 2.0*pi;
    while(angle_diff < -pi) angle_diff += 2.0*pi;
	base.angle_ref.z() = foot[0].angle_ref.z() + angle_diff/2.0;

    base.ori_ref   = FromRollPitchYaw(base.angle_ref);

    // set support foot position
    foot[sup].pos_ref     = stb0.foot_pos[sup];
    foot[sup].angle_ref   = stb0.foot_angle[sup];
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

    // set swing foot position
    if(!stb0.stepping || (timer.time - stb0.tbegin) < dsp_duration)
    {
        foot[swg].pos_ref     = stb0.foot_pos  [swg];
        foot[swg].angle_ref   = stb0.foot_angle[swg];
        foot[swg].ori_ref     = stb0.foot_ori  [swg];
        foot[swg].contact_ref = true;
    }
    else{
        double ts   = timer.time - (stb0.tbegin + dsp_duration);            //< time elapsed in ssp
        double tauv = stb0.duration - dsp_duration; //< duration of vertical movement
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

        // foot turning
        Vector3 turn = stb1.foot_angle[swg] - stb0.foot_angle[swg];
        while(turn.z() >  pi) turn.z() -= 2.0*pi;
        while(turn.z() < -pi) turn.z() += 2.0*pi;

        // foot tilting
        Vector3 tilt = stb0.foot_ori[swg]*Vector3(0.0, swing_tilt, 0.0);

        foot[swg].pos_ref      = (1.0 - ch)*stb0.foot_pos[swg] + ch*stb1.foot_pos[swg];
        foot[swg].pos_ref.z() += (cv*(swing_height + 0.5*descend_depth) - cv2*descend_depth);
        foot[swg].angle_ref    = stb0.foot_angle[swg] + ch*turn + cw*tilt;
        foot[swg].ori_ref      = FromRollPitchYaw(foot[swg].angle_ref);
        foot[swg].contact_ref  = false;

        /*
        // adjust swing foot considering base link inclination
        // base link orientation error
        Quaternion base_rot = 
            FromRollPitchYaw(Vector3(base.angle_ref.x(), base.angle_ref.y(), base.angle_ref.z()))*
            FromRollPitchYaw(Vector3(base.angle    .x(), base.angle    .y(), base.angle_ref.z())).conjugate();

        // modify swing foot pose so that
        // relative position from support foot is rotated as much as base link orientation error
        foot[swg].pos_ref   = base_rot*(foot[swg].pos_ref - foot[sup].pos_ref) + foot[sup].pos_ref;
        foot[swg].ori_ref   = base_rot* foot[swg].ori_ref;
        foot[swg].angle_ref = ToRollPitchYaw(foot[swg].ori_ref);
        */
    }	
}

void SteppingController::AdjustTiming(const Timer& timer, const Param& param, const Centroid& centroid_pred, const Footstep& footstep, Footstep& footstep_buffer){
    const Step& st0  = footstep.steps[0];
    const Step& st1  = footstep.steps[1];
    Step& stb0 = footstep_buffer.steps[0];
    Step& stb1 = footstep_buffer.steps[1];
    int sup =  st0.side;
    int swg = !st0.side;

    // no adjustment during dsp or foot is not lifted
    if(!stb0.stepping || timer.time <= stb0.tbegin + dsp_duration){
        return;
    }
    // no adjustment right before landing
    if(timer.time > stb0.tbegin + stb0.duration - 0.1){
        return;
    }
	
    Vector3 land_min(-0.3, (swg == 0 ? -0.5 : 0.1), 0.0);
    Vector3 land_max( 0.3, (swg == 0 ? -0.1 : 0.5), 0.0);
    Vector3 dcm_offset = st0.foot_ori[sup].conjugate()*(st1.dcm - st1.foot_pos[swg]);
    Vector3 dcm_min = land_min + dcm_offset;
    Vector3 dcm_max = land_max + dcm_offset;

    // check if predicted DCM at landing is inside feasible region
    Vector3 dcm_local = st0.foot_ori[sup].conjugate()*(centroid_pred.dcm_ref - st0.foot_pos[sup]);
    bool inside = true;
    for(int j = 0; j < 2; j++){
        inside &= (dcm_min[j] <= dcm_local[j] && dcm_local[j] <= dcm_max[j]);
    }
    
    if(inside){
        stb0.duration = 0.5*(stb0.duration + st0.duration);
    }
    else{
        stb0.duration = std::max(stb0.duration*0.99, 0.3);
    }
    stb0.duration = std::max(stb0.duration, timer.time - stb0.tbegin);
}

}
}
