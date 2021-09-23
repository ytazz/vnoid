#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

class Robot;
class Footstep;

/* 
 * simple footstep planner
 * 
 */

class FootstepPlanner{
public:
    
public:
	void Plan(const Robot& robot, Footstep& footstep);
    
	FootstepPlanner();
};

}
}
