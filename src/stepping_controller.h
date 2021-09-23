#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

class Robot;
class Footstep;

class SteppingController{
public:
    double       standby_period;      ///< period of initial standby mode
	double       standby_com_height;  ///< com height in standby mode

	int          step_count;
	Step         cur_step[2];
	double       tswitch;

	double    base_ori;
	double    base_angvel;
	Vector3   com_pos_ref;
	Vector3   com_vel_ref;
	Vector3   com_acc_ref;
	Vector3   zmp_ref;
	Vector3   zmp_diff;
	Vector3   dcm_ref;
	Vector3   dcm_diff;

public:
	void Init     (const Robot& robot, const Footstep& footstep);
	void FromRobot();
	void ToRobot  ();
    
	SteppingController();
};

}
}
