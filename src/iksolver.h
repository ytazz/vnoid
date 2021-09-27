#pragma once

#include <cnoid/EigenTypes>

#include <vector>
using namespace std;

namespace cnoid{
namespace vnoid{

class Param;
class Centroid;
class Base;
class Hand;
class Foot;
class Joint;

/* 
 * 
 * computes inverse kinematics for "standard" kinematic model.
 * see accompanying document for derivation.
 * 
 */
class IkSolver{
public:
	void CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q);
    void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q);

    void Comp(const Param& param, const Centroid& centroid, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint);
};

}
}