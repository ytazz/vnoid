#pragma once

#include "robot_base.h"

#include <mujoco/mujoco.h>

namespace cnoid{
namespace vnoid{

/**
 * Robot
 **/
class RobotMujoco : public RobotBase{
public:
	// internal objects
    mjModel*  m;
    mjData*   d;
	
public:
	/**
	 * @brief performs initialization
	 *
	 * This function performs initialization of the base link, the joints, and the sensors.
	 * It assumes that joint.size() is equal to the number of joints of the robot model.
	 * 
	 * @param _m     Mujoco Model
	 * @param _d     Mujoco Data struct
	 * @param timer  timer object to reset
	 * @param joint  array of joint to be configured
	 **/
	void  Init   (mjModel* _m, mjData* _d, const Param& param, Timer& timer, vector<Joint>& joint);

	/**
	 * @brief performs sensing
	 * 
	 * This function performs basic sensing. It retrieves raw data from the acceleration and rate gyro sensors attached to the base link
	 *  and force sensors attached to the feet.
	 * It also reads the current angle and velocity of each joint.
	 * These raw data are filtered and stored in the respective objects passed to this function.
	 * 
	 * @param timer  timer object used to reference dt
	 * @param base   base link object. accleration and angular velocity are stored.
	 * @param foot   array of foot objects. force and moment values are stored.
	 * @param joint  array of joint objects. current angle and velocity are stored.
	 * 
	 **/
	void  Sense  (Timer& timer, Base& base, vector<Joint>& joint);
	void  Sense  (Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint);

	/**
	 * @brief performs actuation
	 * 
	 * This function performs actuation.
	 * 
	 * @param timer  Timer object used to reference dt
	 * @param base   Base link object. 
	 *               If base_actuation is true, its reference position and orientation are directly reflected to the pose of the base link in simulation.
	 *   			 Otherwise, it is not used.
	 * @param joint  Array of joint objects. 
	 *               The torque command of each joint is calculated and commanded to the simulator.
	 * 
	 **/
	void  Actuate(Timer& timer, Base& base, vector<Joint>& joint);

	RobotMujoco();
};

}
}