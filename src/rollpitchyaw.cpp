#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

static const double pi = M_PI;

Vector3 ToRollPitchYaw(const Quaternion& q){
	Vector3 angles;

	Vector3 xdir = q*Vector3::UnitX();
	angles[2] = atan2( xdir.y(), xdir.x());
	angles[1] = atan2(-xdir.z(), sqrt(xdir.x()*xdir.x() + xdir.y()*xdir.y()));
	
	Quaternion qroll = AngleAxis(-angles[1], Vector3::UnitY()) * AngleAxis(-angles[2], Vector3::UnitZ()) * q;
	angles[0] = 2.0 * atan2(qroll.y(), qroll.x());

	// yaw angle needs wrapping
	if(angles[0] >  pi) angles[0] -= 2.0*pi;
	if(angles[0] < -pi) angles[0] += 2.0*pi;

	return angles;
}

Quaternion FromRollPitchYaw(const Vector3& angles){
	return AngleAxis(angles[2], Vector3::UnitZ())
         * AngleAxis(angles[1], Vector3::UnitY())
         * AngleAxis(angles[0], Vector3::UnitX());
}

}
}