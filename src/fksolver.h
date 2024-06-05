#pragma once

#include "types.h"

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
 * computes forward kinematics for "standard" kinematic model.
 * see accompanying document for derivation.
 * 
 */
class FkSolver{
public:
    /** @brief FK from hip to ankle. Returns position, orientation, and axis direction of all intermediate links.
     * 
     *  @param l1   upper leg (thigh) length
     *  @param l2   lower leg (shank) length
     *  @param q    pointer to array of leg joint angles
     *  @param pos  array to store position of links relative to hip
     *  @param ori  array to store orientation of links relative to hip
     *  @param axis array to store axis direction of links in hip local coordinate
     * 
     **/
    void CompLegFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis);
    
    /** @brief FK from hip to ankle. Returns position and orientation of ankle only.
     *
     *  @param l1   upper leg (thigh) length
     *  @param l2   lower leg (shank) length
     *  @param q    pointer to array of leg joint angles
     *  @param pos  ankle position relative to hip
     *  @param ori  ankle orientation relative to hip
     * 
     *  This function computes FK (forward kinematics) of 6-DoF leg.
     *  It assumes a Y-R-P-P-P-R serial link model that is most common in humanoid robots.
     *  Also, it assumes that the 3 axes of the hip all cross at the hip origin,
     *  and the 2 axes of the ankle cross at the ankle origin.
     **/
	void CompLegFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori);
    
    /** @brief Compute leg Jacobian
     *
     *  @param l1   upper leg (thigh) length
     *  @param l2   lower leg (shank) length
     *  @param q    pointer to array of leg joint angles
     *  @param J    6 x 6 Jacobi matrix
     * 
     **/
    void CompLegJacobian(double l1, double l2, const double* q, Eigen::Matrix<double,6,6>& J);

    /** @brief FK from shoulder to wrist. Returns position, orientation, and axis direction of all intermediate links.
     * 
     *  @param l1     upper arm (shoulder to elbow) length
     *  @param l2     lower arm (elbow to wrist) length
     *  @param q      pointer to array of leg joint angles
     *  @param pos    array to store position of links relative to shoulder
     *  @param ori    array to store orientation of links relative to shoulder
     *  @param axis   array to store axis direction of links in shoulder local coordinate
     * 
     **/
    void CompArmFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis);

    /** @brief FK from shoulder to wrist
     *
     *  @param l1     upper arm (shoulder to elbow) length
     *  @param l2     lower arm (elbow to wrist) length
     *  @param q      pointer to array of arm joint angles
     *  @param pos    wrist position relative to shoulder
     *  @param ori    wrist orientation relative to shoulder
     * 
     *  This function computes FK (forward kinematics) of 7-DoF arm.
     *  It assumes a P-R-Y-P-Y-P-R serial link model
     *  where the first 3 axes (P,R,Y) cross at the shoulder origin
     *  and the last 3 axes (Y,P,R) cross at the wrist origin.
     **/
    void CompArmFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori);

    /** @brief Whole-body FK
     *  @param  param     Parameters.
     *  @param  joint     Array of joints. Joint::q is used as input joint angle.
     *  @param  base      Reference to Base object. Base::pos and Base::ori are used as inputs.
     *  @param  centroid  Reference to Centroid object. Center-of-mass is stored to Centroid::com_pos;
     *  @param  hand      Reference to Hand objects. Computed position and orientation are stored.
     *  @param  foot      Reference to Foot objects. Computed position and orientation are stored.
     *
     *  This function computes whole-body FK.
     *  It takes base position, orientation and joint position as inputs and computes absolute position/orientation of hands and feet.
     *  It also computes center-of-mass.
     *  Note that current implementation assumes that joints of body trunk (i.e., chest links) are fixed to 0.
     **/
    void Comp(const Param& param, const vector<Joint>& joint, const Base& base, Centroid& centroid, vector<Hand>& hand, vector<Foot>& foot);

};

}
}