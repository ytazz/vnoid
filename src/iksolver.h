#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

/* 
 * 
 * computes inverse kinematics for "standard" kinematic model.
 * see accompanying document for derivation.
 * 
 */
class IkSolver{
public:
	void CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, vector<double>& q);
    void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, vector<double>& q);

};&

}
}