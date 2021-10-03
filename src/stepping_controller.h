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
    double    dsp_duration;

    double    tswitch;   //< time instant of previous support foot exchange

public:
	void Init  (const Param& param, Centroid& centroid, Base& base);
	void Update(const Timer& timer, const Param& param, Footstep& footstep, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	SteppingController();
};

}
}
