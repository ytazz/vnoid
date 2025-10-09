#pragma once

#include <cnoid/EigenTypes>

#include "stabilizer.h"

#include <vector>
using namespace std;

/* 
 * A controller that implements DCM-modulation based walking
 * Its functionality and design are mostly the same as the stepping controller included in vnoidlib
 * The main differences are:
 * - it uses desired DCM offset for walking instead of desired footsteps
 * - it realizes yaw regulation in addition to roll/pitch regulation 
 */

namespace cnoid{
namespace vnoid{

class Param;
class Timer;
class Centroid;
class Base;
class Foot;

class ReactiveWalkingController : public Stabilizer{
public:
    double    swing_height;
    double    swing_tilt;
    double    nominal_duration;
    double    min_duration;
    double    max_dcm_distance;
    double    dsp_rate;
    double    descend_rate;
    double    descend_depth;
    double    stride;
    double    sway;
    double    spacing;
    double    turn;
    Vector3   orientation_ctrl_gain_p;
	Vector3   orientation_ctrl_gain_d;
    Vector3   orientation_ctrl_gain_i;
    Vector3   orientation_ctrl_deadband;
    bool      enable_logging;
    
    Vector2   dcm_offset[2], dcm_offset_mod;
    bool      stepping;
    int       sup, swg, nstep;
    Vector3   lift_pos, lift_angle;
    Vector3   land_pos, land_angle;
    Vector3   land_dcm;
    double    time_switch;
    double    t_now, t_land, ttl, duration;
    double    yaw_angle_des, yaw_angvel_des;
    Vector3   angle_switch;
    Vector3   Ldmax;
    Vector3   Kp;
    Vector3   Kd;
    Vector3   Ki;
    Vector3   I;
    Vector3   delta, delta2;
    Vector3   Ld_local, theta, theta_i, omega, omegadd, theta_ref, omega_ref;
   
    FILE* fileLogFk;
    FILE* fileLogIk;
    FILE* fileLogCtrl;
    FILE* fileLogStepFk[2];
    FILE* fileLogStepIk[2];

public:
    void   CalcDcmOffset(const Param& param);
	double CalcLanding  (const Param& param, const Base& base, const Centroid& centroid, const vector<Foot>& foot, double t_land_adjust);
    void   Update(const Timer& timer, const Param& param, Centroid& centroid, Base& base, vector<Foot>& foot);
    void   SaveFootstep(const Param& param, const vector<Foot>& foot, int idx, bool fk_or_ik);
    
	ReactiveWalkingController();
};

}
}
