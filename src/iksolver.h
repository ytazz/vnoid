#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

class Robot;

class IkSolver{
public:
	virtual void Init() = 0;
	virtual void Comp(Robot* robot, double dt) = 0;

};

}
}