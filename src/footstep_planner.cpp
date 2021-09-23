#include "footstep.h"
#include "footstep_planner.h"
#include "robot.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double eps = 1.0e-10;

///////////////////////////////////////////////////////////////////////////////////////////////////

FootstepPlanner::FootstepPlanner(){
}

void FootstepPlanner::Plan(const Robot& robot, Footstep& footstep){
    Step& st0 = footstep[0];
	st0.side = 0;
	st0.foot_pos[0] = Vector3(0.0, -st0.spacing/2.0, 0.0);
	st0.foot_pos[1] = Vector3(0.0,  st0.spacing/2.0, 0.0);
	st0.foot_ori[0] = 0.0;
	st0.foot_ori[1] = 0.0;

    int nstep = footstep.size();
	for(int i = 0; i < nstep-1; i++){
	    Step& st0 = footstep[i+0];
	    Step& st1 = footstep[i+1];

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

	    st1.side = !st0.side;
	    st1.foot_pos[sup] = st0.foot_pos[sup];
	    st1.foot_ori[sup] = st0.foot_ori[sup];
	    st1.foot_pos[swg] = st0.foot_pos[sup] + Eigen::AngleAxisd(st0.foot_ori[sup], Vector3::UnitZ())*dprel;
	    st1.foot_ori[swg] = st0.foot_ori[sup] + dtheta;
	}

    // generate reference dcm and zmp 
	double T = sqrt(robot.com_height/robot.gravity);
		
	// set final step's state
	int i = nstep-1;
	footstep[i].zmp = (footstep[i].foot_pos[0] + footstep[i].foot_pos[1])/2.0;
	footstep[i].dcm = (footstep[i].foot_pos[0] + footstep[i].foot_pos[1])/2.0;
	i--;

	// calc N-1 to 0 step's state
	for( ; i >= 0; i--) {
		int sup =  footstep[i].side;
		int swg = !footstep[i].side;
		
		double a = exp(-footstep[i].duration/T);
		// for initial step, set dcm to middle of feet, and determine zmp
		if(i == 0){
			//stepAuxs[i].dcm = (footstep[i].footPos[sup] + footstep[i].footPos[swg])/2.0;
			footstep[i].zmp = (footstep[i].dcm - a*footstep[i+1].dcm)/(1.0 - a);
		}
		// for other steps, set zmp to support foot, and determine dcm
		else{
			footstep[i].zmp = footstep[i].foot_pos[sup];
			footstep[i].dcm = footstep[i].zmp + a*(footstep[i+1].dcm - footstep[i].zmp);
		}
	}
}

}
}
