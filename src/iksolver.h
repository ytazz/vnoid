#pragma once

#include <cnoid/EigenTypes>

#include <vector>
using namespace std;

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
	static void CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q);
    static void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q);

};

}
}