#pragma once

#include <cnoid/EigenTypes>

#include "footstep.h"

namespace cnoid{
namespace vnoid{

class Robot;
class Footstep;

class SteppingController{
public:
	int       step_index;
	Step      cur_step[2];
	double    tswitch;

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
	void FromRobot(const Robot& robot, Footstep& footstep);
	void ToRobot  (Robot& robot, const Footstep& footstep);
    void CalcSwing(double t, Vector3& pos, double& ori, Vector3& vel, double& angvel);
    
	SteppingController();
};

}
}
