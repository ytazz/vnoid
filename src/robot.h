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

class Joint{
public:
	double  pgain;
	double  dgain;
	double  ulimit;

	double  q;
	double  dq;
	double  q_ref;
	double  dq_ref;
	double  u;

    void Set(double _pgain, double _dgain, double _ulimit);
	void CalcTorque(double dt);

	Joint();
};

class Base{
public:
	Vector3     pos_ref;
	Vector3     angle;
	Vector3     angle_ref;
	Quaternion  ori_ref;
	Vector3     vel_ref;
	Vector3     angvel;
	Vector3     angvel_ref;
	Vector3     acc_ref;
	Vector3     angacc_ref;
		
	Base();
};

class Hand{	
public:
	Vector3    pos_ref;
	Vector3    vel_ref;
	Vector3    acc_ref;
	Quaternion ori_ref;
	Vector3    angle_ref;
	Vector3    angvel_ref;
	Vector3    angacc_ref;

    Hand();
};

class Foot{
public:
	bool        contact;
	bool        contact_ref;
	double      contact_duration;
	double      balance;
	double      balance_ref;
	Vector3     pos_ref;
	Quaternion  ori_ref;
	Vector3     angle_ref;
	Vector3     vel_ref;
	Vector3     angvel_ref;
	Vector3     acc_ref;
	Vector3     angacc_ref;
	Vector3     force;
	Vector3     force_ref;
	Vector3     force_error;
	Vector3     moment;
	Vector3     moment_ref;
	Vector3     moment_error;
	Vector3     zmp;
	Vector3     zmp_ref;
	Vector3     dpos;
	Vector3     drot;
	double      tliftoff;
	double      tlanding;
	Vector3     foothold;   ///< foot origin to foothold origin
	
	Foot();
};

class Centroid{
public:
	Vector3  force_ref;
	Vector3  moment_ref;
	Vector3  zmp;
	Vector3  zmp_ref;
	Vector3  dcm_ref;

	Vector3  com_pos_ref;
	Vector3  com_vel_ref;
	Vector3  com_acc_ref;
	
	Centroid();
};

/*
  physical parameters
 */
class Param{
public:
    // dynamical parameters
	double   total_mass;
	double   com_height;
	double   gravity;
    double   T;
    
    // kinematic parameters
    Vector3   base_to_shoulder[2];
    Vector3   base_to_hip[2];
    Vector3   wrist_to_hand[2];
    Vector3   ankle_to_foot[2];
    int       arm_joint_index[2];
    int       leg_joint_index[2];
    double    upper_arm_length;
    double    lower_arm_length;
    double    upper_leg_length;
    double    lower_leg_length;

    void Init();

    Param();
};

class Timer{
public:
    double   dt;
	int      count;
	double   time;

    void  Countup();

    Timer();
};

class Robot{
public:
	Body*     io_body;

    bool      base_actuation;
    double    gyro_filter_cutoff;
    double    acc_filter_cutoff;
    double    foot_force_filter_cutoff;
    double    foot_moment_filter_cutoff;
    double    joint_pos_filter_cutoff;
	
    AccelerationSensor* accel_sensor;
	RateGyroSensor*     gyro_sensor;
	ForceSensor*        foot_force_sensor[2];

    Filter   acc_filter [3];
    Filter   gyro_filter[3];
    Filter   foot_force_filter [2][3];
    Filter   foot_moment_filter[2][3];
    vector<Filter>  joint_pos_filter;

	string   base_force_sensor_name;
    string   base_acc_sensor_name;
	string   base_gyro_sensor_name;

    Vector3  gyro_axis_x;
    Vector3  gyro_axis_y;
    Vector3  gyro_axis_z;

    string   right_force_sensor_name;
    string   left_force_sensor_name;
    	
public:
	void  Init   (SimpleControllerIO* io, Timer& timer, vector<Joint>& joint);
	void  Sense  (Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint);
	void  Actuate(Timer& timer, Base& base, vector<Joint>& joint);

	Robot();
};

}
}