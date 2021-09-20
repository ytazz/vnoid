#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/** 
   conversion between roll-pitch-yaw and quaternion
   q = Rz(yaw)*Ry(pitch)*Rx(roll)
 **/

// quaternion to roll-pitch-yaw
vec3_t ToRollPitchYaw  (const quat_t& q     );

// roll-pitch-yaw to quaternion
quat_t FromRollPitchYaw(const vec3_t& angles);

// angular velocity to time derivative of roll-pitch-yaw
vec3_t VelocityToRollPitchYaw  (const vec3_t& angvel);

// time derivative of roll-pitch-yaw to angular velocity
vec3_t VelocityFromRollPitchYaw(const vec3_t& vel   );

// quaternion to axis-angle
vec3_t ToAxisAngle(const quat_t& q);

// axis-angle to quaternion
quat_t FromAxisAngle(const vec3_t& v);

}
