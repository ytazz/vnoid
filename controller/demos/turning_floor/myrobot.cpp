#include "myrobot.h"
#include "rollpitchyaw.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    // fix base
    base_actuation = true;
}

void MyRobot::Init(SimpleControllerIO* io){
    // joints for turn table
    joint.resize(3);
    joint[ 0].Set(10000.0, 1000.0, 1000.0);  //< z: yaw
    joint[ 1].Set(10000.0, 1000.0, 1000.0);  //< y: pitch
    joint[ 2].Set(10000.0, 1000.0, 1000.0);  //< x: roll

    Robot::Init(io, timer, joint);
}

void MyRobot::Control(){
    base.pos_ref = Vector3(0.0, 0.0, 0.0);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);

    Robot::Sense(timer, base, joint);

    // tilting angle
    double tilt = 0.05;

    // tilting direction
    double theta = 2.0*std::max(timer.time - 2.0, 0.0);

    Quaternion q(Eigen::AngleAxisd(tilt, Vector3(cos(theta), sin(theta), 0.0)));

    // transform it to roll-pitch-yaw
    Vector3 angle = ToRollPitchYaw(q);

    joint[0].q_ref = angle.z();
    joint[1].q_ref = angle.y();
    joint[2].q_ref = angle.x();
    
    Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
