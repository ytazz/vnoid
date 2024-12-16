#pragma once

#include "types.h"

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
    double    timing_adaptation_weight;

    bool      buffer_ready;
    double    time_to_landing;

public:
	void Update      (const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot);
    
	SteppingController();
};

}
}
