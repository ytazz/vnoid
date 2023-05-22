#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = true;
}

void MyRobot::Init(SimpleControllerIO* io){
    // joints for moving table
    joint.resize(2);
    joint[ 0].Set(1000.0, 1000.0, 1000.0);
    joint[ 1].Set(1000.0, 1000.0, 1000.0);

    Robot::Init(io, timer, joint);
}

void MyRobot::Control(){
    base.pos_ref = Vector3(0.0, 0.0, 0.0);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);

    Robot::Sense(timer, base, joint);

    /*
     * p =  A sin(omega t)
     * v =  A omega cos(omega t)
     * a = -A omega^2 sin(omega t)
     * 
     * max acceleration = A omega^2 = 1.8
     */
    joint[0].q_ref = 0.3*sin(1.0*(std::max(timer.time - 2.0, 0.0)));
    joint[1].q_ref = 0.3*cos(1.0*(std::max(timer.time - 2.0, 0.0)));

    Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
