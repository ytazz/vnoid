#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = false;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    param.com_height = 0.7;
    
    // kinematic parameters
    param.base_to_shoulder[0] = Vector3(0.0, -0.1,  0.3);
    param.base_to_shoulder[1] = Vector3(0.0,  0.1,  0.3);
    param.base_to_hip     [0] = Vector3(0.0, -0.1, -0.1);
    param.base_to_hip     [1] = Vector3(0.0,  0.1, -0.1);
    param.wrist_to_hand   [0] = Vector3(0.0,  0.0, -0.1);
    param.wrist_to_hand   [1] = Vector3(0.0,  0.0, -0.1);
    param.ankle_to_foot   [0] = Vector3(0.0,  0.0, -0.05);
    param.ankle_to_foot   [1] = Vector3(0.0,  0.0, -0.05);
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

    InitMarkers(io);

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // comp FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);
    
    // generate IK inputs. edit as you like! //
	
    const double pi = 3.1415926535;
    double theta = 1.0*std::max(timer.time - 2.0, 0.0);

    foot[0].pos_ref = Vector3(0.0, -0.1, 0.0);
    foot[1].pos_ref = Vector3(0.0,  0.1, 0.0);
    
    hand[0].pos_ref   = Vector3(0.0, -0.2,  0.7);
    hand[0].ori_ref   = AngleAxis(-pi/2.0, Vector3::UnitX());
    
    hand[1].pos_ref   = Vector3(0.0,  0.2,  0.7);
    hand[1].ori_ref   = AngleAxis( pi/2.0, Vector3::UnitX());

    centroid.com_pos_ref = Vector3(0.0*sin(theta), 0.0*sin(theta), param.com_height + 0.07*sin(theta));
    
    // comp CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

    Robot::Actuate(timer, base, joint);

    UpdateMarkers(base, centroid, hand, foot);
    
	timer.Countup();
}


}
}
