#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

class Stabilizer{
public:
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

public:
    void Step(Robot& robot, Centroid& centroid, Base& base, Foot& foot[2]);
    
	Stabilizer();
};

}
}
