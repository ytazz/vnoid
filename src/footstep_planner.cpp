#include "footstep.h"
#include "footstep_planner.h"
#include "robot_base.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double eps = 1.0e-10;

///////////////////////////////////////////////////////////////////////////////////////////////////

FootstepPlanner::FootstepPlanner(){
}

void FootstepPlanner::Plan(const Param& param, Footstep& footstep){
    
    // we assume that foot placement, support foot flag, and dcm of step[0] are specified from outside

    // determine foot placement and support foot flag of remaining steps
    int nstep = footstep.steps.size();
	for(int i = 0; i < nstep-1; i++){
	    Step& st0 = footstep.steps[i+0];
	    Step& st1 = footstep.steps[i+1];

	    int sup =  st0.side;
	    int swg = !st0.side;

	    double  dtheta = st0.turn;
	    double  l      = st0.stride;
        double  d      = st0.sway;
	    double  w      = (sup == 0 ? 1.0 : -1.0) * st0.spacing;
	    double  dz     = st0.climb;
	    Vector3 dprel;

	    if(std::abs(dtheta) < eps){
		    dprel = Vector3(l, w + d, dz);
	    }
	    else{
		    double r = l/dtheta;
		    dprel = Vector3(
			    (r - w/2.0 - d)*sin(dtheta),
			    (r + w/2.0) - (r - w/2.0 - d)*cos(dtheta),
			    dz);
	    }

	    // support foot exchange
	    st1.side = !st0.side;

	    // support foot pose does not change
        st1.foot_pos  [sup] = st0.foot_pos  [sup];
	    st1.foot_angle[sup] = st0.foot_angle[sup];
	    st1.foot_ori  [sup] = st0.foot_ori  [sup];

        // swing foot pose changes
	    st1.foot_pos  [swg] = st0.foot_pos  [sup] + st0.foot_ori[sup]*dprel;
        st1.foot_angle[swg] = st0.foot_angle[sup] + Vector3(0.0, 0.0, dtheta);
	    st1.foot_ori  [swg] = FromRollPitchYaw(st1.foot_angle[swg]);
	}
}

void FootstepPlanner::AlignToGround(const Ground& ground, Footstep& footstep){
	// pivot with respect to the initial support foot center
	Vector3 pivot = footstep.steps[0].foot_pos[footstep.steps[0].side];

    // ground normal vector
    Vector3 normal = ground.ori * Vector3(0.0, 0.0, 1.0);

	// 0-th step is unchanged
	for(int k = 0; k < footstep.steps.size(); k++){
		Step& st = footstep.steps[k];

		for(int i = 0; i < 2; i++){
			// modify z
			Vector3 dp = st.foot_pos[i] - pivot;
			dp.z() = -(normal.x()*dp.x() + normal.y()*dp.y())/normal.z();

			st.foot_pos[i].z() = pivot.z() + dp.z();

			// ground normal in yaw-local coordinate of footprint
			Vector3 nl = AngleAxis(-st.foot_angle[i].z(), Vector3::UnitZ()) * normal;
			st.foot_angle[i].x() = asin (-nl.y());
			st.foot_angle[i].y() = atan2( nl.x(), nl.z());

            // convert it back to quaternion
			st.foot_ori[i] = FromRollPitchYaw(st.foot_angle[i]);
		}
	}
}

void FootstepPlanner::GenerateDCM(const Param& param, Footstep& footstep){
    // generate reference dcm and zmp 
	// dcm of step[0] should be already specified

    int nstep = footstep.steps.size();
		
	// set final step's state
	int i = nstep-1;
	// ZMP is in the middle of feet
	footstep.steps[i].zmp = (footstep.steps[i].foot_pos[0] + footstep.steps[i].foot_pos[1])/2.0;

	// DCM is com_height above ZMP
	footstep.steps[i].dcm = (footstep.steps[i].foot_pos[0] + footstep.steps[i].foot_pos[1])/2.0
		                  + Vector3(0.0, 0.0, param.com_height);
	i--;

	// calc N-1 to 0 step's state
	for( ; i >= 0; i--) {
		Step& st0 = footstep.steps[i+0];
		Step& st1 = footstep.steps[i+1];
			
		int sup =  st0.side;
		int swg = !st0.side;
		
		double a = exp(-st0.duration/param.T);
		// for initial step, the dcm is specified from outside. determine zmp accordingly
		if(i == 0){
			st0.zmp = (st0.dcm - a*st1.dcm)/(1.0 - a) - Vector3(0.0, 0.0, param.com_height);
		}
		// for other steps
		else{
			const double eps = 1.0e-3;
			// if swing foot position is not changing, treated as double support and set zmp to the middle of feet
			if( (st0.foot_pos  [swg] - st1.foot_pos  [swg]).norm() < eps &&
				(st0.foot_angle[swg] - st1.foot_angle[swg]).norm() < eps ){
				st0.zmp = (st0.foot_pos[sup] + st0.foot_pos[swg])/2.0;
			}
			// otherwise, set zmp to support foot
			else{
				st0.zmp = st0.foot_pos[sup];
			}

			// determine dcm from zmp
			st0.dcm = (1.0 - a)*(st0.zmp + Vector3(0.0, 0.0, param.com_height)) + a*st1.dcm;
		}

		// set stepping flag based on whether swing foot position is chanching or not
		if( (st0.foot_pos  [swg] - st1.foot_pos  [swg]).norm() < eps &&
			(st0.foot_angle[swg] - st1.foot_angle[swg]).norm() < eps ){
			st0.stepping = false;
		}
		else{
			st0.stepping = true;
		}

	}
}

}
}
