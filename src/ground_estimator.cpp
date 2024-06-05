#include "ground_estimator.h"

#include "robot_base.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

GroundEstimator::GroundEstimator(){
    correction_constant = 10.0;
    correction_limit    = 0.0;
}

void GroundEstimator::Update(const Timer& timer, const Base& base, const vector<Foot>& foot, Ground& ground){
    // do nothing if the robot is not in contact with the ground
    if(!foot[0].contact && !foot[1].contact)
        return;
    
    // estimate ground normal based on rotation angles of feet contacting the ground
    // expecting that feet are well aligned with the ground slope
    Vector3 n;
    if( foot[0].contact && !foot[1].contact){
        n = FromRollPitchYaw(Vector3(foot[0].angle.x(), foot[0].angle.y(), foot[0].angle.z()))*Vector3(0.0, 0.0, 1.0);
    }
    if(!foot[0].contact &&  foot[1].contact){
        n = FromRollPitchYaw(Vector3(foot[1].angle.x(), foot[1].angle.y(), foot[1].angle.z()))*Vector3(0.0, 0.0, 1.0);
    }
    if( foot[0].contact &&  foot[1].contact){
        // z direction of each foot with respect to global coordinate frame
        Vector3 nf[2];
        nf[0] = FromRollPitchYaw(Vector3(foot[0].angle.x(), foot[0].angle.y(), foot[0].angle.z()))*Vector3(0.0, 0.0, 1.0);
        nf[1] = FromRollPitchYaw(Vector3(foot[1].angle.x(), foot[1].angle.y(), foot[1].angle.z()))*Vector3(0.0, 0.0, 1.0);

        // take weighted average
        n = foot[0].balance*nf[0] + foot[1].balance*nf[1];
    }
    
    // calculate tilt (roll and pitch)
    Vector3 tilt;
    tilt.x() = asin (-n.y());
    tilt.y() = atan2( n.x(), n.z());

    ground.angle.x() += -(ground.angle.x() - tilt.x()) * (timer.dt/correction_constant);
    ground.angle.y() += -(ground.angle.y() - tilt.y()) * (timer.dt/correction_constant);

    ground.angle.x() = std::min(std::max(-correction_limit, ground.angle.x()), correction_limit);
    ground.angle.y() = std::min(std::max(-correction_limit, ground.angle.y()), correction_limit);
		
    ground.ori = FromRollPitchYaw(ground.angle);

    // calculate difference of robot's heading and gradient of slope
    Vector3 ng      = ground.ori*Vector3(0.0, 0.0, 1.0);
    ground.tilt     = atan2(sqrt(ng.x()*ng.x() + ng.y()*ng.y()), ng.z());
    ground.gradient = atan2(-ng.y(), -ng.x());
}

}
}
