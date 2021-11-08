#pragma once

#include <cnoid/EigenTypes>

#include <deque>
using namespace std;

namespace cnoid{
namespace vnoid{

/**
 Single footstep
 **/
struct Step{
	double   stride;
    double   sway;
	double   spacing;
	double   turn;
	double   climb;
	double   duration;
    
	int      side;
		
	Vector3  foot_pos   [2];
	double   foot_ori   [2];
	Vector3  foot_vel   [2];
	double   foot_angvel[2];
    Vector3  zmp;
	Vector3  dcm;

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