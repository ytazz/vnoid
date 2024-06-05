#pragma once

#include "types.h"

namespace cnoid{

/// vnoid namespace
namespace vnoid{

    /** 
     *  @file
     *  conversion between roll-pitch-yaw and quaternion
     *  q = Rz(yaw)*Ry(pitch)*Rx(roll)
     **/

    /// quaternion to roll-pitch-yaw
    Vector3 ToRollPitchYaw(const Quaternion& q);

    /// roll-pitch-yaw to quaternion
    Quaternion FromRollPitchYaw(const Vector3& angles);

}
}