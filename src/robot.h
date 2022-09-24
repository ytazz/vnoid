#pragma once

#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include "filter.h"

#include <string>
using namespace std;

namespace cnoid{
namespace vnoid{

/**
 *  A single joint of a robot 
 **/
class Joint{
public:
	double  pgain;    ///< pgain of PD controller
	double  dgain;    ///< dgain of PD controller
	double  ulimit;

	double  q;        ///< current joint angle
	double  dq;       ///< current joint velocity
	double  q_ref;    ///< desired joint angle
	double  dq_ref;   ///< desired joint velocity
	double  u;        ///< torque command
	double  u_ref;

	/**
	 * Set joint parameters
	 **/
    void Set(double _pgain, double _dgain, double _ulimit);

	/**
	 * Calculate joint torque command using PD control law plus reference torque
	 **/
	void CalcTorque();

	Joint();
};

/**
 * Base link 
 **/
class Base{
public:
	Vector3     pos;        ///< position
	Vector3     pos_ref;    ///< reference position
	Vector3     angle;      ///< current orientation in roll-pitch-yaw
	Vector3     angle_ref;  ///< reference orientation in roll-pitch-yaw
	Quaternion  ori;        ///< orientation in quaternion
	Quaternion  ori_ref;    ///< reference orientation in quaternion
	Vector3     vel_ref;    ///< reference velocity
	Vector3     angvel;     ///< current angular velocity
	Vector3     angvel_ref; ///< reference angular velocity
	Vector3     acc_ref;    ///< reference acceleration
	Vector3     angacc_ref; ///< reference angular acceleration
		
	Base();
};

/**
 * Hand
 **/
class Hand{	
public:
	Vector3    pos;        ///< position
	Vector3    pos_ref;    ///< reference position
	Vector3    vel_ref;    ///< reference velocity
	Vector3    acc_ref;    ///< reference acceleration
	Quaternion ori;        ///< orientation in quaternion
	Quaternion ori_ref;    ///< reference orientation in quaternion
	Vector3    angle;      ///< orientation in roll-pitch-yaw
	Vector3    angle_ref;  ///< reference orientation in roll-pitch-yaw
	Vector3    angvel_ref; ///< reference angular velocity
	Vector3    angacc_ref; ///< reference angular acceleration
	double     arm_twist;  ///< desired angle of shoulder-yaw joint. used by iksolver

    Hand();
};

class Foot{
public:
	bool        contact;      ///< current contact state (true if foot is in contact with the ground)
	bool        contact_ref;  ///< reference contact state
	double      balance;      ///< current balance ratio [0.0, 1.0].  indicates the ratio of vertical reaction force applied to this foot
	double      balance_ref;  ///< reference balance ratio [0.0, 1.0]
	Vector3     pos;          ///< position
	Vector3     pos_ref;      ///< reference position
	Quaternion  ori;          ///< orientation in quaternion
	Quaternion  ori_ref;      ///< reference orientation in quaternion
	Vector3     angle;        ///< orientation in roll-pitch-yaw
	Vector3     angle_ref;    ///< reference orientation in roll-pitch-yaw
	Vector3     vel_ref;      ///< reference velocity
	Vector3     angvel_ref;   ///< reference angular velocity
	Vector3     acc_ref;      ///< reference acceleration
	Vector3     angacc_ref;   ///< reference angular acceleration
	Vector3     force;        ///< ground reaction force acting on this foot
	Vector3     force_ref;    ///< reference ground reaction force
	Vector3     moment;       ///< ground reaction moment acting on this foot
	Vector3     moment_ref;   ///< reference ground reaction moment
	Vector3     zmp;          ///< ZMP (i.e., center-of-pressure) of this foot
	Vector3     zmp_ref;      ///< reference ZMP of this foot
	
	Foot();
};

/**
 * Centroid: center-of-mass and ZMP
 **/
class Centroid{
public:
	Vector3  force_ref;    ///< reference force
	Vector3  moment_ref;   ///< reference moment
	Vector3  zmp;          ///< current ZMP
	Vector3  zmp_ref;      ///< reference ZMP
	Vector3  dcm;
	Vector3  dcm_ref;      ///< reference DCM (divergent component of motion)

	Vector3  com_pos;
	Vector3  com_pos_ref;  ///< reference position of CoM
	Vector3  com_vel_ref;  ///< reference velocity of CoM
	Vector3  com_acc_ref;  ///< reference acceleration of CoM
	
	Centroid();
};

/**
    Ground plane
 **/
class Ground{
public:
    Vector3     angle;   ///< ground inclination angle in roll and pitch. yaw is always zero
    Quaternion  ori;     ///< ground inclination
	double      tilt;
	double      gradient;

	Ground();
};

/**
 * Physical parameters
 **/
class Param{
public:
    // dynamical parameters
	double   total_mass;   ///< total mass
	double   com_height;   ///< height of CoM from the ground
	double   gravity;      ///< gravitational acceleration
    double   T;            ///< constant of LIPM. T= sqrt(h/g)
    
    // kinematic parameters
    Vector3   base_to_shoulder[2];   ///< relative position of each shoulder (i.e., base of arm) w.r.t. base link origin
    Vector3   base_to_hip[2];        ///< relative position of each hip (i.e., base of leg) w.r.t. base link origin
    Vector3   base_to_com;           ///< nominal relative position of CoM w.r.t. base link origin
    Vector3   wrist_to_hand[2];      ///< offset from wrist joint to hand center
    Vector3   ankle_to_foot[2];      ///< offset from ankle joint to foot center
    double    upper_arm_length;      ///< length of upper arm (i.e., shoulder to elbow)
    double    lower_arm_length;      ///< length of lower arm (i.e., elbow to wrist)
    double    upper_leg_length;      ///< length of upper leg (i.e., hip to knee)
    double    lower_leg_length;      ///< length of lower leg (i.e., knee to ankle)

	// mass parameters
	double    trunk_mass;
	double    arm_mass[7];           ///< array of mass of links
	double    leg_mass[6];
	Vector3   trunk_com;             ///< array of center of mass of links
	Vector3   arm_com[7];
	Vector3   leg_com[6];

	int       arm_joint_index[2];    ///< index of the first joint of each arm
    int       leg_joint_index[2];    ///< index of the first joint of each leg

   	// dynamic parameters
	Vector3   zmp_min;
	Vector3   zmp_max;

	/**
	 * Set default values to parameters 
	 **/
    void Init();

    Param();
};

/**
 * Timer
 **/
class Timer{
public:
    double   dt;      ///< control period
	int      count;   ///< counter that is counted up at every control cycle
	double   time;    ///< current time

	/** @brief  Calling this function increments the counter and updates current time.
	 *
	 **/
    void  Countup();

    Timer();
};

/**
 * Robot
 **/
class Robot{
public:
	// parameters
	
	bool      base_actuation;             ///< base actuation. if set true, the base link of the robot can be moved directly.
    double    gyro_filter_cutoff;         ///< cutoff frequency [Hz] of filter for rate gyro sensor
    double    acc_filter_cutoff;          ///< cutoff frequency [Hz] of filter for acceleration sensor
    double    foot_force_filter_cutoff;   ///< cutoff frequency [Hz] of filter for force sensor attached to each foot
    double    foot_moment_filter_cutoff;  ///< cutoff frequency [Hz] of filter for moment sensor attached to each foot
    double    joint_pos_filter_cutoff;    ///< cutoff frequency [Hz] of joint angle filter. common for all joints
	
	string   base_acc_sensor_name;    ///< name of acceleration sensor. specify the name you set in the .body file of the robot
	string   base_gyro_sensor_name;   ///< name of rate gyro sensor.  specify the name you set in the .body file of the robot
    string   right_force_sensor_name; ///< name of force sensor attached to the right foot. specify the name you set in the .body file of the robot
    string   left_force_sensor_name;  ///< name of force sensor attached to the left foot. specify the name you set in the .body file of the robot

    Vector3  gyro_axis_x;  ///< direction of x-axis of the gyro sensor in the base link coordinate.
    Vector3  gyro_axis_y;  ///< direction of y-axis of the gyro sensor in the base link coordinate.
    Vector3  gyro_axis_z;  ///< direction of z-axis of the gyro sensor in the base link coordinate.

	// internal objects

	Body*     io_body;   ///< handle to IO Body of Choreonoid

	AccelerationSensor* accel_sensor;          ///< handle to acceleration sensor of Choreonoid
	RateGyroSensor*     gyro_sensor;           ///< handle to rate gyro sensor of Choreonoid
	ForceSensor*        foot_force_sensor[2];  ///< handle to force sensro of Choreonoid attached to each foot

    Filter   acc_filter [3];            ///< acceleration filter (x|y|z)
    Filter   gyro_filter[3];            ///< rate gyro filter (x|y|z)
    Filter   foot_force_filter [2][3];  ///< force filter of each foot (r|l, x|y|z)
    Filter   foot_moment_filter[2][3];  ///< moment filter of each foot (r|l, x|y|z)
    vector<Filter>  joint_pos_filter;   ///< joint angle filter
    	
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