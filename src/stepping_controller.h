#pragma once

#include <cnoid/EigenTypes>

#include "footstep.h"

#include <vector>
using namespace std;

namespace cnoid{
namespace vnoid{

class Param;
class Timer;
class Footstep;
class Centroid;
class Base;
class Foot;

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
	void Init     (const Param& param, const Footstep& footstep);
	void CalcSwing(double t, Vector3& pos, double& ori, Vector3& vel, double& angvel);
    void Update   (const Timer& timer, const Param& param, const Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	SteppingController();
};

}
}
