#pragma once

#include "types.h"

#include <vector>
using namespace std;

namespace cnoid{
namespace vnoid{

class Timer;
class Base;
class Foot;
class Ground;

/**
    Ground plane estimator.
    Estimates ground slope based on base link and/or foot inclination

 **/
class GroundEstimator{
public:
    /* @brief Time constant of ground slope estimation
     * 
     **/
    double correction_constant;

    /* @brief Upper limit of estimate ground inclination
     * 
     **/
    double correction_limit;

public:
    /* @brief Update ground slope estimation
     * 
     * Current implementation estimates ground slope based on the inclination of feet contacting the ground.
     * 
     **/
	void Update(const Timer& timer, const Base& base, const vector<Foot>& foot, Ground& ground);
    

	GroundEstimator();
};

}
}
