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

    bool      buffer_ready;

public:
	bool CheckLanding(const Timer& timer, Step& step, vector<Foot>& foot);
	void Update      (const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot);
    void AdjustTiming(const Timer& timer, const Param& param, const Centroid& centroid_pred, const Footstep& footstep, Footstep& footstep_buffer);
    
	SteppingController();
};

}
}
