#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

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

    param.trunk_mass = 24.0;
    param.trunk_com = Vector3(0.0, 0.0, 0.166);

    param.arm_mass[0] = 0.5;
    param.arm_mass[1] = 0.5;
    param.arm_mass[2] = 1.0;
    param.arm_mass[3] = 0.5;
    param.arm_mass[4] = 1.0;
    param.arm_mass[5] = 0.5;
    param.arm_mass[6] = 0.5;
    param.arm_com[0] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[1] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[2] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[3] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[4] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[5] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[6] = Vector3(0.0, 0.0,  0.0);

    param.leg_mass[0] = 0.5;
    param.leg_mass[1] = 0.5;
    param.leg_mass[2] = 1.5;
    param.leg_mass[3] = 1.5;
    param.leg_mass[4] = 0.5;
    param.leg_mass[5] = 0.5;
    param.leg_com[0] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[1] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[2] = Vector3(0.0, 0.0, -0.15);
    param.leg_com[3] = Vector3(0.0, 0.0, -0.20);
    param.leg_com[4] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[5] = Vector3(0.0, 0.0,  0.0);

    param.zmp_min = Vector3(-0.1, -0.05, 0.0);
    param.zmp_max = Vector3( 0.1,  0.05, 0.0);
    
    param.Init();

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    joint[ 0].Set(1000.0, 200.0, 100.0);
    joint[ 1].Set(1000.0, 200.0, 100.0);
    joint[ 2].Set(1000.0, 200.0, 100.0);
    joint[ 3].Set(1000.0, 200.0, 100.0);
    joint[ 4].Set(1000.0, 200.0, 100.0);
    joint[ 5].Set(1000.0, 200.0, 100.0);
    joint[ 6].Set(1000.0, 200.0, 100.0);
    joint[ 7].Set(1000.0, 200.0, 100.0);
    joint[ 8].Set(1000.0, 200.0, 100.0);
    joint[ 9].Set(1000.0, 200.0, 100.0);
    joint[10].Set(1000.0, 200.0, 100.0);
    joint[11].Set(1000.0, 200.0, 100.0);
    joint[12].Set(1000.0, 200.0, 100.0);
    joint[13].Set(1000.0, 200.0, 100.0);
    joint[14].Set(1000.0, 200.0, 100.0);
    joint[15].Set(1000.0, 200.0, 100.0);
    joint[16].Set(1000.0, 200.0, 100.0);
    joint[17].Set(1000.0, 200.0, 100.0);
    joint[18].Set(1000.0, 200.0, 100.0);
    joint[19].Set(1000.0, 200.0, 100.0);
    joint[20].Set(1000.0, 200.0, 100.0);
    joint[21].Set(1000.0, 200.0, 100.0);
    joint[22].Set(100.0, 20.0, 100.0);
    joint[23].Set(100.0, 20.0, 100.0);
    joint[24].Set(1000.0, 200.0, 100.0);
    joint[25].Set(1000.0, 200.0, 100.0);
    joint[26].Set(1000.0, 200.0, 100.0);
    joint[27].Set(1000.0, 200.0, 100.0);
    joint[28].Set(100.0, 20.0, 100.0);
    joint[29].Set(100.0, 20.0, 100.0);
    
    // init hardware (simulator interface)
    gyro_filter_cutoff = 100.0;
	Robot::Init(io, timer, joint);

    // init stabilizer
    stabilizer.orientation_ctrl_gain_p  = 500.0;
    stabilizer.orientation_ctrl_gain_d  = 50.0;
        
    /*
    stabilizer.gain <<
   -0.0007,   -0.0000,   -0.0002,   -0.0000,    0.0100,    0.0000,    0.1418,    0.0000,   -0.0000,    0.0006,   -0.0000,    0.0002,
   -0.0000,   -0.0007,   -0.0000,   -0.0002,   -0.0000,    0.0100,   -0.0000,    0.1418,   -0.0006,    0.0000,   -0.0002,    0.0000,
    0.0000,  -12.9132,    0.0000,   -1.6935,   -0.0000,    0.0002,   -0.0000,    0.0023,    8.1660,   -0.0000,    5.0284,   -0.0000,
   12.9132,   -0.0000,    1.6935,   -0.0000,   -0.0002,    0.0000,   -0.0023,    0.0000,   -0.0000,    8.1660,   -0.0000,    5.0284,
   -0.0000,   -1.8255,   -0.0000,   -0.4127,   -0.0000,   -0.0000,   -0.0000,   -0.0004,   -1.5776,    0.0000,   -0.4105,    0.0000,
    1.8255,   -0.0000,    0.4127,   -0.0000,    0.0000,   -0.0000,    0.0004,   -0.0000,   -0.0000,   -1.5776,   -0.0000,   -0.4105;
    */
    stabilizer.min_contact_force        = 1.0;
    stabilizer.force_ctrl_damping       = 5.0;
    stabilizer.force_ctrl_gain          = 0.000;
    stabilizer.force_ctrl_limit         = 0.01;
    stabilizer.moment_ctrl_damping      = 5.0;
    stabilizer.moment_ctrl_gain         = 0.000;
    stabilizer.moment_ctrl_limit        = 0.5;

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

    base.ori_ref   = Quaternion(1.0, 0.0, 0.0, 0.0);
    base.angle_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_pos_ref = Vector3(0.0, 0.0, 0.55);
    centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
    centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
    foot[0].pos_ref = Vector3(0.02, -0.15, 0.0);
    foot[1].pos_ref = Vector3(0.02,  0.15, 0.0);
    foot[0].contact_ref = true;
    foot[1].contact_ref = true;
    
    stabilizer.Update(timer, param, centroid, base, foot);

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;
    
    // comp CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

    joint[1].q_ref = stabilizer.phi_mod.y();
    
    Robot::Actuate(timer, base, joint);

    // update marker poses for visualization
    io_body->link(marker_index + 4)->p() = hand[0].pos_ref;
    io_body->link(marker_index + 4)->R() = hand[0].ori_ref.matrix();
    io_body->link(marker_index + 5)->p() = hand[1].pos_ref;
    io_body->link(marker_index + 5)->R() = hand[1].ori_ref.matrix();
    io_body->link(marker_index + 6)->p() = foot[0].pos_ref;
    io_body->link(marker_index + 6)->R() = foot[0].ori_ref.matrix();
    io_body->link(marker_index + 7)->p() = foot[1].pos_ref;
    io_body->link(marker_index + 7)->R() = foot[1].ori_ref.matrix();
	io_body->link(marker_index + 8)->p() = centroid.zmp;
	io_body->link(marker_index + 9)->p() = centroid.zmp_ref;
    
	timer.Countup();
}


}
}
