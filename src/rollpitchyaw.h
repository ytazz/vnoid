#pragma once

#include <cnoid/EigenTypes>

namespace cnoid{
namespace vnoid{

/** 
   conversion between roll-pitch-yaw and quaternion
   q = Rz(yaw)*Ry(pitch)*Rx(roll)
 **/

// quaternion to roll-pitch-yaw
Vector3 ToRollPitchYaw(const Quaternion& q);

// roll-pitch-yaw to quaternion
Quaternion FromRollPitchYaw(const Vector3& angles);

}
}