#pragma once

#include "types.h"

#include <vector>
using namespace std;

#include "robot_base.h"

namespace cnoid{
namespace vnoid{

class FkSolver;

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

    /// old implementation which turned out to be incorrect. left here for educational purpose...
    void CompLegIkOld(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q);

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
     *  To resolve redundancy, CompArmIk requires the user to specify the yaw angle of the upper arm.
     *  CompArmIk2 requires the desired joint axis direction of the elbow joint.
     **/
    void CompElbowAngle(const Vector3& pos, double l1, double l2, double* q);
    void CompWristAngles(const Quaternion& ori, double* q);
    void CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q);
    void CompArmIk2(const Vector3& pos, const Quaternion& ori, double l1, double l2, const Vector3& elbow_y, double* q);

    /** @brief Whole-body IK
     *
     *  @param  param     Parameters
     *  @param  base      Reference to Base object. Base::pos_ref and Base::ori_ref are used as inputs.
     *  @param  hand      Reference to Hand objects. Hand::pos_ref and Hand::ori_ref are used as inputs.
     *  @param  foot      Reference to Foot objects. Foot::pos_ref and Foot::ori_ref are used as inputs.
     *  @param  joint     Array of joints to store the result
     * 
     *  Simple whole-body IK.
     *  It takes reference pose (position and orientation) of the base link, hand, and feet
     *  as inputs and computes desired joint angles.
     *  Offsets (base link to shoulders and hips, wrist to hand, ankle to foot) set in param are taken into consideration.
     *
     **/
    void Comp(const Param& param, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint);

    /** @brief Whole-body CoM IK
     *
     *  @param  fk_solver Pointer to FK solver to be used internally
     *  @param  param     Parameters
     *  @param  centroid  Reference to Centroid object. Centroid::com_pos_ref is used as an input.
     *  @param  base      Reference to Base object. Base::ori_ref is used as an input, and Base::pos_ref is used to store the result.
     *  @param  hand      Reference to Hand objects. Hand::pos_ref and Hand::ori_ref are used as inputs.
     *  @param  foot      Reference to Foot objects. Foot::pos_ref and Foot::ori_ref are used as inputs.
     *  @param  joint     Array of joints to store the result
     * 
     *  CoM IK.
     *  It takes reference CoM position, reference base link orientation, and reference pose of the hand and feet
     *  as inputs and computes desired base link position and desired joint angles.
     *  Offsets (base link to shoulders and hips, wrist to hand, ankle to foot) set in param are taken into consideration.
     * 
     *  It internally computes whole-body IK and FK repeatedly while adjusting the base link position until
     *  CoM position computed by FK matches the reference value.
     *
     **/
    void Comp(FkSolver* fk_solver, const Param& param, Centroid& centroid, Base& base, vector<Hand>& hand, vector<Foot>& foot, vector<Joint>& joint);

protected:
    // variables for internal use
    Base           base_tmp;
    Centroid       centroid_tmp;
    vector<Joint>  joint_tmp;
    vector<Hand>   hand_tmp;
    vector<Foot>   foot_tmp;

};

}
}