#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    // base link is fixed to space
    base_actuation = true;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    
    // kinematic parameters
    param.base_to_shoulder[0] = Vector3(0.0, -0.1,  0.3);
    param.base_to_shoulder[1] = Vector3(0.0,  0.1,  0.3);
    param.base_to_hip     [0] = Vector3(0.0, -0.1, -0.1);
    param.base_to_hip     [1] = Vector3(0.0,  0.1, -0.1);
    param.wrist_to_hand   [0] = Vector3(0.0,  0.0, -0.0);
    param.wrist_to_hand   [1] = Vector3(0.0,  0.0, -0.0);
    param.ankle_to_foot   [0] = Vector3(0.0,  0.0, -0.0);
    param.ankle_to_foot   [1] = Vector3(0.0,  0.0, -0.0);
    param.arm_joint_index [0] =  4;
    param.arm_joint_index [1] = 11;
    param.leg_joint_index [0] = 18;
    param.leg_joint_index [1] = 24;
    param.upper_arm_length = 0.2;
    param.lower_arm_length = 0.2;
    param.upper_leg_length = 0.3;
    param.lower_leg_length = 0.4;

    param.Init();

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    joint[ 0].Set(5000.0, 200.0, 100.0);
    joint[ 1].Set(5000.0, 200.0, 100.0);
    joint[ 2].Set(5000.0, 200.0, 100.0);
    joint[ 3].Set(5000.0, 200.0, 100.0);
    joint[ 4].Set(5000.0, 200.0, 100.0);
    joint[ 5].Set(5000.0, 200.0, 100.0);
    joint[ 6].Set(5000.0, 200.0, 100.0);
    joint[ 7].Set(5000.0, 200.0, 100.0);
    joint[ 8].Set(5000.0, 200.0, 100.0);
    joint[ 9].Set(5000.0, 200.0, 100.0);
    joint[10].Set(5000.0, 200.0, 100.0);
    joint[11].Set(5000.0, 200.0, 100.0);
    joint[12].Set(5000.0, 200.0, 100.0);
    joint[13].Set(5000.0, 200.0, 100.0);
    joint[14].Set(5000.0, 200.0, 100.0);
    joint[15].Set(5000.0, 200.0, 100.0);
    joint[16].Set(5000.0, 200.0, 100.0);
    joint[17].Set(5000.0, 200.0, 100.0);
    joint[18].Set(5000.0, 200.0, 100.0);
    joint[19].Set(5000.0, 200.0, 100.0);
    joint[20].Set(5000.0, 200.0, 100.0);
    joint[21].Set(5000.0, 200.0, 100.0);
    joint[22].Set(5000.0, 200.0, 100.0);
    joint[23].Set(5000.0, 200.0, 100.0);
    joint[24].Set(5000.0, 200.0, 100.0);
    joint[25].Set(5000.0, 200.0, 100.0);
    joint[26].Set(5000.0, 200.0, 100.0);
    joint[27].Set(5000.0, 200.0, 100.0);
    joint[28].Set(5000.0, 200.0, 100.0);
    joint[29].Set(5000.0, 200.0, 100.0);
    
    // init hardware (simulator interface)
	Robot::Init(io, timer, joint);

    // configure marker links
    marker_index = 31;
    num_markers  = 10;
    for (int i = 0; i < num_markers; i++) {
        io_body->link(marker_index + i)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(marker_index + i));
    }

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // comp FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);
    
    // generate IK inputs. edit as you like! //
	
    const double pi = 3.1415926535;
    double theta = 2.0*timer.time;

    base.pos_ref = Vector3(0.0, 0.0, 1.5);
    base.ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);

    foot[0].pos_ref = Vector3( 0.3*cos(-theta), -0.1, 1.5 - 0.7 + 0.3*sin(-theta));
    foot[0].ori_ref = base.ori_ref;
    
    foot[1].pos_ref = Vector3(-0.3*cos(-theta),  0.1, 1.5 - 0.7 - 0.3*sin(-theta));
    foot[1].ori_ref = base.ori_ref;
    
    hand[0].pos_ref   = base.pos_ref + base.ori_ref*Vector3( 0.3+0.2*cos(-theta), -0.2,  0.2*sin(-theta));
    hand[0].ori_ref   = AngleAxis(-pi/2.0, Vector3::UnitX());
    hand[0].arm_twist = pi/2.0;
    
    hand[1].pos_ref   = base.pos_ref + base.ori_ref*Vector3( 0.3+0.2*cos(-theta),  0.2,  0.2*sin(-theta));
    hand[1].ori_ref   = AngleAxis( pi/2.0, Vector3::UnitX());
    hand[1].arm_twist = -pi/2.0;
    
    ik_solver.Comp(param, base, hand, foot, joint);

    Robot::Actuate(timer, base, joint);

    // update marker poses for visualization
    io_body->link(marker_index + 0)->p() = hand[0].pos;
    io_body->link(marker_index + 0)->R() = hand[0].ori.matrix();
    io_body->link(marker_index + 1)->p() = hand[1].pos;
    io_body->link(marker_index + 1)->R() = hand[1].ori.matrix();
    io_body->link(marker_index + 2)->p() = foot[0].pos;
    io_body->link(marker_index + 2)->R() = foot[0].ori.matrix();
    io_body->link(marker_index + 3)->p() = foot[1].pos;
    io_body->link(marker_index + 3)->R() = foot[1].ori.matrix();
    io_body->link(marker_index + 4)->p() = hand[0].pos_ref;
    io_body->link(marker_index + 4)->R() = hand[0].ori_ref.matrix();
    io_body->link(marker_index + 5)->p() = hand[1].pos_ref;
    io_body->link(marker_index + 5)->R() = hand[1].ori_ref.matrix();
    io_body->link(marker_index + 6)->p() = foot[0].pos_ref;
    io_body->link(marker_index + 6)->R() = foot[0].ori_ref.matrix();
    io_body->link(marker_index + 7)->p() = foot[1].pos_ref;
    io_body->link(marker_index + 7)->R() = foot[1].ori_ref.matrix();

	
	timer.Countup();
}


}
}
