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
    // IK from hip to ankle
	void CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q);

    // IK from shoulder to wrist
    void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q);

    // whole body IK
    // current implementation assumes that the CoM coinsides the base link origin
    void Comp(const Param& param, const Centroid& centroid, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint);
};

}
}