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
    double    swing_height;
    double    swing_tilt;
    double    dsp_duration;
    double    descend_duration;
    double    descend_depth;
    double    landing_adjust_rate;

    double    tswitch;   //< time instant of previous support foot exchange
    double    zdiff;
    bool      adjusted;

public:
	void Init        (const Param& param, Centroid& centroid, Base& base);
    bool CheckLanding(const Timer& timer, Footstep& footstep, vector<Foot>& foot);
	void Update      (const Timer& timer, const Param& param, Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	SteppingController();
};

}
}
