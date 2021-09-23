#pragma once

#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include <string>
using namespace std;

namespace cnoid{
namespace vnoid{

class IkSolver;
class FootstepPlanner;

class Robot{
public:
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
		ForceSensor*        forceSensor;
		AccelerationSensor* accelSensor;
		RateGyroSensor*     gyroSensor;

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
		ForceSensor*  forceSensor;

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
		Vector3     cop;
		Vector3     cop_ref;
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

		Vector3 com_pos_ref;
		Vector3 com_pos_cor;
		Vector3 com_pos_mod;
		Vector3 com_vel_ref;
		Vector3 com_vel_cor;
		Vector3 com_vel_mod;
		Vector3 com_acc_ref;
		Vector3 com_acc_cor;

		Centroid();
	};

	Body*    ioBody;

    bool     baseActuation;
	int      joystickCycle;
	Joystick joystick;

    FootstepPlanner*  planner;
	IkSolver*         iksolver;

	double  dt;
	int     count;
	double  time;

	Centroid             centroid;
	Base                 base;
	Hand                 hand[2];
	Foot                 foot[2];
	std::vector<Joint>   joint;

    string  baseForceSensorName;
    string  baseAccSensorName;
	string  baseGyroSensorName;
    Vector3 gyroAxisX;
    Vector3 gyroAxisY;
    Vector3 gyroAxisZ;
    string  rightForceSensorName;
    string  leftForceSensorName;
    int     downsample;                //< down-sampling interval for file saving
	string  logFilename;
	
	Vector3 nominal_inertia;
	double  total_mass;
	double  com_height;
	double  gravity;
    double  T;
	double  com_pos_gain;
	double  com_vel_gain;

	double  min_contact_force;
	double  force_ctrl_damping;
	double  force_ctrl_gain;
	double  force_ctrl_limit;
	double  moment_ctrl_damping;
	double  moment_ctrl_gain;
	double  moment_ctrl_limit;
	double  orientation_ctrl_gain_p;
	double  orientation_ctrl_gain_d;
	double  swing_height_adjust_rate;
	
	FILE*   logFile;
	
public:
	void CompFZmp();
	void CompIZmp();
	void Countup ();

	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Sense  ();
	virtual void  Control();
	virtual void  Save   ();

	Robot();

};

}
}