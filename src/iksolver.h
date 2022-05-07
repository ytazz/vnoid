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
    /** @brief IK from hip to ankle
     *
     *  @param pos  desired ankle position relative to hip
     *  @param ori  desired ankle orientation relative to hip
     *  @param l1   upper leg (thigh) length
     *  @param l2   lower leg (shank) length
     *  @param q    pointer to array of leg joint angles
     * 
     *  This function computes IK (inverse kinematics) of 6-DoF leg.
     *  It assumes a Y-R-P-P-P-R serial link model that is most common in humanoid robots.
     *  Also, it assumes that the 3 axes of the hip all cross at the hip origin,
     *  and the 2 axes of the ankle cross at the ankle origin.
     **/
	void CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q);

    /** @brief IK from shoulder to wrist
     *
     *  @param pos    desired wrist position relative to shoulder
     *  @param ori    desired wrist orientation relative to shoulder
     *  @param l1     upper arm (shoulder to elbow) length
     *  @param l2     lower arm (elbow to wrist) length
     *  @param q2_ref angle of upper arm yaw
     *  @param q      pointer to array of arm joint angles
     * 
     *  This function computes IK (inverse kinematics) of 7-DoF arm.
     *  It assumes a P-R-Y-P-Y-P-R serial link model
     *  where the first 3 axes (P,R,Y) cross at the shoulder origin
     *  and the last 3 axes (Y,P,R) cross at the wrist origin.
     *  To resolve redundancy, it simply requires the user to specify the yaw angle of the upper arm.
     *  6 other joint angles are computed by the function.
     **/
    void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q);

    /** @brief Whole-body IK
     *
     *  @param  param     parameters
     *  @param  centroid  reference CoM position
     *  @param  base      reference base orientation
     *  @param  hand      reference hand pose
     *  @param  foot      reference foot pose
     *  @param  joint     array of joints to store the result
     * 
     *  Simple whole-body IK.
     *  It takes reference base orientation, reference CoM position, and reference pose (position and orientation) of feet and hands
     *  as inputs and computes desired joint angles.
     *  Offsets (base link to shoulders and hips, wrist to hand, ankle to foot) set in param are taken into consideration.
     *  Note that current implementation assumes that the CoM coincides with the base link origin.
     **/
    void Comp(const Param& param, const Centroid& centroid, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint);

};

}
}