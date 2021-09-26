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
    double    swing_height;

public:
	void Init     (const Param& param, const Footstep& footstep, Centroid& centroid, Base& base);
	void CalcSwing(double t, Vector3& pos, double& ori, Vector3& vel, double& angvel);
    void Update   (const Timer& timer, const Param& param, const Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	SteppingController();
};

}
}
