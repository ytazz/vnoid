#pragma once

#include <cnoid/EigenTypes>

#include <deque>
using namespace std;

namespace cnoid{
namespace vnoid{

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

    Step();
};

class Footstep{
public:
    deque<Step>  steps;

};

}
}