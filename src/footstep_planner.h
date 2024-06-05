#pragma once

#include "types.h"

namespace cnoid{
namespace vnoid{

class Param;
class Footstep;
class Ground;

/* 
 * simple footstep planner
 * 
 */

class FootstepPlanner{
public:
    
public:
    /*
     * @brief Generate footsteps
     * 
     * Note: Call GenerateDCM for generating zmp and dcm reference.
     */
	void Plan(const Param& param, Footstep& footstep);
    
    /**
     *  @brief Align footsteps to ground plane
     **/
    void AlignToGround(const Ground& ground, Footstep& footstep);

    /**
     *  @brief Generate dcm and zmp reference
     **/
    void GenerateDCM(const Param& param, Footstep& footstep);

	FootstepPlanner();
};

}
}
