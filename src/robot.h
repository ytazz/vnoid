#pragma once

#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/SimpleController>

#include "robot_base.h"

#include <string>
using namespace std;

namespace cnoid{
namespace vnoid{

/**
 * Robot
 **/
class Robot : public RobotBase{
public:
	string   base_acc_sensor_name;    ///< name of acceleration sensor. specify the name you set in the .body file of the robot
	string   base_gyro_sensor_name;   ///< name of rate gyro sensor.  specify the name you set in the .body file of the robot
    string   right_force_sensor_name; ///< name of force sensor attached to the right foot. specify the name you set in the .body file of the robot
    string   left_force_sensor_name;  ///< name of force sensor attached to the left foot. specify the name you set in the .body file of the robot

    // internal objects
    Body*     io_body;   ///< handle to IO Body of Choreonoid

	AccelerationSensor* accel_sensor;          ///< handle to acceleration sensor of Choreonoid
	RateGyroSensor*     gyro_sensor;           ///< handle to rate gyro sensor of Choreonoid
	ForceSensor*        foot_force_sensor[2];  ///< handle to force sensro of Choreonoid attached to each foot

public:
	/**
	 * @brief performs initialization
	 *
	 * This function performs initialization of the base link, the joints, and the sensors.
	 * It assumes that joint.size() is equal to the number of joints than can be accessed through the I/O object.
	 * 
	 * @param io     simple controller I/O object passed from Choreonoid
	 * @param timer  timer object to reset
	 * @param joint  array of joint to be configured
	 **/
	void  Init   (SimpleControllerIO* io, Timer& timer, vector<Joint>& joint);

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

	Robot();
};

}
}