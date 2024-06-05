#pragma once

#include "types.h"

#include <deque>
using namespace std;

namespace cnoid{
namespace vnoid{

/**
 Single footstep
 **/
class Step{
public:
	double   stride;    ///< longitudinal step length
    double   sway;      ///< lateral step length
	double   spacing;   ///< lateral spacing between left and right feet
	double   turn;      ///< turning angle in single step
	double   climb;     ///< vertical displacement in single step
	double   duration;  ///< step duration
    
	int      side;      ///< indicates which foot (0: right, 1: left) is the support foot in this step
	bool     stepping;  ///< stepping or standing still
	double   tbegin;    ///< starting time of this step
		
	Vector3     foot_pos   [2];  ///< position of each foot at the beginning of this step
	Vector3     foot_angle [2];
	Quaternion  foot_ori   [2];  ///< orientation of each foot at the beginning of this step
	Vector3     foot_vel   [2];
	Vector3     foot_angvel[2];
    Vector3     zmp;             ///< position of zmp during this step
	Vector3     dcm;             ///< position of dcm at the beginning of this step

    Step(double _stride = 0.0, double _sway = 0.0, double _spacing = 0.0, double _turn = 0.0, double _climb = 0.0, double _duration = 0.5, int _side = 0);
};

/**
 Footstep sequence
 **/
class Footstep{
public:
    /// series of footsteps
    deque<Step>  steps;
};

}
}