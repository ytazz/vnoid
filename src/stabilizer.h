#pragma once

#include "types.h"

#include <vector>
using namespace std;

namespace cnoid{
namespace vnoid{

class Timer;
class Param;
class Centroid;
class Base;
class Foot;
class Footstep;
class Step;

/** @brief Simple balance controller
 *
 **/
class Stabilizer{
public:
	/// parameters for ground reaction force control
    double  min_contact_force;         ///< vertical force threshold to detect contact
	double  force_ctrl_damping;        ///< damping coefficient of force damping control
	double  force_ctrl_gain;           ///< gain of force damping control
	double  force_ctrl_limit;          ///< movement limit of force damping control
	double  moment_ctrl_damping;       ///< damping coefficient of moment damping control
	double  moment_ctrl_gain;          ///< gain of moment damping control
	double  moment_ctrl_limit;         ///< movement limit of moment damping control
	
	/* @brief feedback gain
	 * 
	 * PD Gains used for balance control using ZMP as control input.
	 */
	double  orientation_ctrl_gain_p;
	double  orientation_ctrl_gain_d;

	double  dcm_ctrl_gain;
	double  recovery_moment_limit;
	double  dcm_deviation_limit;
	double  base_tilt_rate;
	double  base_tilt_damping_p;
	double  base_tilt_damping_d;

	Vector3     dpos[2];   ///< foot position modification
	Vector3     drot[2];   ///< foot orientation modification

public:
	void CalcBaseTilt(const Timer& timer, const Param& param, Base& base, Vector3 theta, Vector3 omega);
	
	/**
	 *  @brief calculates 1-step update of DCM dynamics.
	 * 
	 *  This is an internal function called from Update and Predict.
	 **/
	void CalcDcmDynamics(const Timer& timer, const Param& param, const Base& base, const vector<Foot>& foot, Vector3 theta, Vector3 omega, Centroid& centroid);

	/** @brief calculates ZMP from ground reaction force acting on the feet
	 *
	 *  @param  param     parameters
	 *  @param  centroid  centroid
	 *  @param  foot      array of feet
	 * 
	 *  For each foot, foot.contact and foot.zmp are calculated.
	 *  Here, the ZMP of each foot is the center of pressure expressed in the **local** coordinate of foot.
	 *  Weight balance in percentage is stored to foot.balance.
	 *  The ZMP is calculated and stored to centroid.zmp, and it is expressed in the **global** coordinate.
	 *  Here, foot.pos_ref and foot.ori_ref are used for calculating the ZMP.
	 *  This means reference foot poses are used instead of actual poses (possibly calculated by forward kinematics).
	 *  Current implementation assumes that there are exactly two feet.
	 **/
    void CalcZmp(const Param& param, Centroid& centroid, vector<Foot>& foot);
    
	/** @brief calculates desired force distribution from desired ZMP
	 *
	 *  @param  param     parameters
	 *  @param  centroid  centroid
	 *  @param  foot      array of feet
	 * 
	 *  Inputs to this function are centroid.zmp_ref (desired ZMP in **global** coordinate) and foot.contact_ref (reference contact state).
	 *  It computes foot.zmp_ref (desired ZMP of each foot in **local** coordinate) and foot.balance_ref (desired weight ratio)
	 *  together with foot.force_ref and foot.moment_ref (desired force and moment, both in **local** coordinate).
	 * 
	 *  In the case of single support (only one foot has contact_ref set true), calculation is straightforward.
	 *  In the case of double support (both feet have contact_ref set true) the method used here is not so sophisticated (beware!).
	 *  First, it determines foot.balance_ref based on the distance between the desired ZMP and reference foot position.
	 *  Next, it determines the desired ZMP of each foot.
	 *  There is no guarantee that the calculatd desired ZMP will be inside the support region.
	 **/
    void CalcForceDistribution(const Param& param, Centroid& centroid, vector<Foot>& foot);
    
	/** @brief Performs single cycle of balance control ground reaction force control
	 *  
	 *  @param  timer
	 *  @param  param
	 *  @param  centroid
	 *  @param  base
	 *  @param  foot
	 *
	 *  
	 **/
    void Update(const Timer& timer, const Param& param, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	Stabilizer();
};

}
}
