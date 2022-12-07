#pragma once

#include "robot.h"
#include "footstep.h"

/*
 * Robot class used for demos
 * 
 * - marker visualization functions
 * 
 */

namespace cnoid{
namespace vnoid{

class RobotDemo : public Robot{
public:
	int  marker_index;
    int  num_markers;

public:
	virtual void  InitMarkers(SimpleControllerIO* io);
	virtual void  UpdateMarkers(const Base& base, const Centroid& centroid, const vector<Hand>& hand, const vector<Foot>& foot);
	virtual void  UpdateMarkers(const Base& base, const Footstep& footstep_buffer);
	
	RobotDemo();

};

}
}