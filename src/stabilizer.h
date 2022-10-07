#pragma once

#include <cnoid/EigenTypes>

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
	
	/* @brief feedback gain (old formulation)
	 * 
	 * PD Gains used for balance control using ZMP as control input.
	 * Use gain matrix for more general balance control.
	 */
	//double  orientation_ctrl_gain_p;   ///< p gain of orientation control
	//double  orientation_ctrl_gain_d;   ///< d gain of orientation control

	double  orientation_ctrl_gain_p;
	double  orientation_ctrl_gain_d;
	double  dcm_ctrl_gain;

	/* @brief feedback gain matrix
	 *
	 * State feedback gain matrix.
	 * State variable is the following 12D vector:
	 *  base link rotation angle x (roll)
	 *  base link rotation angle y (pitch)
	 *  base link angular velocity x (roll)
	 *  base link angular velocity y (pitch)
	 *  upper body rotation angle x (roll)
	 *  upper body rotation angle y (pitch)
	 *  upper body angular velocity x (roll)
	 *  upper body angular velocity y (pitch)
	 *  CoM position x
	 *  CoM position y
	 *  CoM velocity x
	 *  CoM velocity y
	 * Control input is the following 6D vector:
	 *  upper body angular acceleration (roll)
	 *  upper body angular acceleration (pitch)
	 *  CoM acceleration x
	 *  CoM acceleration y
	 *  ZMP x
	 *  ZMP y
	 *
	 * All variables are offset from reference value.
	 *
	 */ 
	//Eigen::Matrix<double,  6, 12>    gain;

	//Eigen::Matrix<double, 12,  1>    state;
	//Eigen::Matrix<double,  6,  1>    input;
	
	/// upper body rotation
	//Vector3     phi_mod;
	//Vector3     phid_mod;
	
	/// CoM modification
	//Vector3     com_pos_mod;
	//Vector3     com_vel_mod;
	//Vector3     dcm_mod;
	//Vector3     zmp_mod;
	
	Vector3     dpos[2];   ///< foot position modification
	Vector3     drot[2];   ///< foot orientation modification

public:

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
    void Update(const Timer& timer, const Param& param, const Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	// do prediction
	void Predict(const Timer& timer, const Param& param, const Footstep& footstep_buffer, const Base& base, Centroid& centroid);
	
	Stabilizer();
};

}
}
