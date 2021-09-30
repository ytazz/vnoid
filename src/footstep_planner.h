#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

class Param;
class Footstep;

/* 
 * simple footstep planner
 * 
 */

class FootstepPlanner{
public:
    
public:
    /*
     * generate footsteps including zmp and dcm
     * 
     */
	void Plan(const Param& param, Footstep& footstep);
    
	FootstepPlanner();
};

}
}
