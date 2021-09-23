#pragma once

#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include <string>
using namespace std;

namespace cnoid{
namespace vnoid{

struct Joint{
	double  pgain;
	double  dgain;
	double  ulimit;

	double  q;
	double  dq;
	double  q_ref;
	double  dq_ref;
	double  u;

	void CalcTorque(double dt);

	Joint();
};

struct Base{
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

struct Hand{	
	Vector3    pos_ref;
	Vector3    vel_ref;
	Vector3    acc_ref;
	Quaternion ori_ref;
	Vector3    angle_ref;
	Vector3    angvel_ref;
	Vector3    angacc_ref;
};

struct Foot{
	bool        contact;
	bool        contact_ref;
	double      contact_duration;
	double      balance;
	double      balance_ref;
	Vector3     pos_ref;
	Vector3     pos_mod;
	Quaternion  ori_ref;
	Vector3     angle_ref;
	Vector3     angle_mod;
	Vector3     vel_ref;
	Vector3     vel_mod;
	Vector3     angvel_ref;
	Vector3     angvel_mod;
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
struct Centroid{
	Vector3  force_ref;
	Vector3  moment_ref;
	Vector3  moment_mod;
	Vector3  zmp;
	Vector3  zmp_ref;
	Vector3  zmp_mod;

	Vector3  com_pos_ref;
	Vector3  com_pos_cor;
	Vector3  com_pos_mod;
	Vector3  com_vel_ref;
	Vector3  com_vel_cor;
	Vector3  com_vel_mod;
	Vector3  com_acc_ref;
	Vector3  com_acc_cor;

	Centroid();
};

/*
  physical parameters
 */
struct Param{
    Vector3  nominal_inertia;
	double   total_mass;
	double   com_height;
	double   gravity;
    double   T;
};

class Robot{
public:
	Body*     ioBody;

    bool      baseActuation;
	int       joystickCycle;
	Joystick  joystick;

    ForceSensor*        forceSensor;
	AccelerationSensor* accelSensor;
	RateGyroSensor*     gyroSensor;
	ForceSensor*        footForceSensor[2];

	double  dt;
	int     count;
	double  time;

	//Centroid             centroid;
	//Base                 base;
	//Hand                 hand[2];
	//Foot                 foot[2];
	//std::vector<Joint>   joint;

    string   baseForceSensorName;
    string   baseAccSensorName;
	string   baseGyroSensorName;
    Vector3  gyroAxisX;
    Vector3  gyroAxisY;
    Vector3  gyroAxisZ;
    string   rightForceSensorName;
    string   leftForceSensorName;
    	
public:
	void  Init   (SimpleControllerIO* io);
	void  Sense  ();
	void  Actuate();
    void  Countup();

	Robot();
};

}
}