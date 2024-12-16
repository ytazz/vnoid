#pragma once

#include "types.h"
#include "filter.h"

#include <string>
#include <vector>
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
	double  ddq;      ///< current joint acceleration
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
	Vector3     vel;        ///< velocity
	Vector3     vel_ref;    ///< reference velocity
	Vector3     angvel;     ///< current angular velocity
	Vector3     angvel_ref; ///< reference angular velocity
	Vector3     acc;
	Vector3     acc_ref;    ///< reference acceleration
	Vector3     angacc;
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
	
	/// IK options
	bool     fix_arm_twist;
    bool     fix_elbow_dir;
    double   arm_twist;     ///< desired angle of shoulder-yaw joint. used by iksolver
    Vector3  elbow_dir;

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
	Vector3  zmp_target;   ///< target ZMP
	Vector3  dcm;          ///< DCM (divergent component of motion)
	Vector3  dcm_ref;      ///< reference DCM
	Vector3  dcm_target;   ///< target DCM

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
	double   total_mass;       ///< total mass
	Vector3  nominal_inertia;  ///< inertia around CoM in x,y,z directions
	double   com_height;       ///< height of CoM from the ground
	double   gravity;          ///< gravitational acceleration
    double   T;                ///< constant of LIPM. T= sqrt(h/g)
    
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

	// control parameters
	int       control_cycle;

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
    int      control_count;
	double   time;    ///< current time

	/** @brief  Calling this function increments the counter and updates current time.
	 *
	 **/
    void  Countup();

    Timer();
};

/**
 * Robot base class
 **/
class RobotBase{
public:
	// parameters
	bool      base_state_from_simulator;  ///< if set true, the base link states are directly retreived from the simulator.
	bool      base_actuation;             ///< base actuation. if set true, the base link of the robot can be moved directly.
    double    gyro_filter_cutoff;         ///< cutoff frequency [Hz] of filter for rate gyro sensor
    double    acc_filter_cutoff;          ///< cutoff frequency [Hz] of filter for acceleration sensor
    double    foot_force_filter_cutoff;   ///< cutoff frequency [Hz] of filter for force sensor attached to each foot
    double    foot_moment_filter_cutoff;  ///< cutoff frequency [Hz] of filter for moment sensor attached to each foot
    double    joint_pos_filter_cutoff;    ///< cutoff frequency [Hz] of joint angle filter. common for all joints
	
	Vector3  gyro_axis_x;  ///< direction of x-axis of the gyro sensor in the base link coordinate.
    Vector3  gyro_axis_y;  ///< direction of y-axis of the gyro sensor in the base link coordinate.
    Vector3  gyro_axis_z;  ///< direction of z-axis of the gyro sensor in the base link coordinate.

	/// filters
    Filter   acc_filter [3];            ///< acceleration filter (x|y|z)
    Filter   gyro_filter[3];            ///< rate gyro filter (x|y|z)
    Filter   foot_force_filter [2][3];  ///< force filter of each foot (r|l, x|y|z)
    Filter   foot_moment_filter[2][3];  ///< moment filter of each foot (r|l, x|y|z)
    vector<Filter>  joint_pos_filter;   ///< joint angle filter
    	
public:
	
	RobotBase();
    virtual ~RobotBase();
};

}
}