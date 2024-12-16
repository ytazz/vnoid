#include "myrobot.h"
#include "rollpitchyaw.h"

#include <cnoid/ExecutablePath>

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    //base_actuation = true;
    base_actuation = false;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
	param.com_height =  0.70;
	param.gravity    =  9.8;

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

    param.zmp_min = Vector3(-0.1, -0.05, -0.1);
    param.zmp_max = Vector3( 0.1,  0.05,  0.1);

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
    stabilizer.orientation_ctrl_gain_p = 100.0;
    stabilizer.orientation_ctrl_gain_d = 10.0;
    stabilizer.dcm_ctrl_gain           = 2.0;
    stabilizer.base_tilt_rate          = 0.0;
    stabilizer.base_tilt_damping_p     = 0.0;
    stabilizer.base_tilt_damping_d     = 0.0;

    // set initial state
    base.ori_ref   = Quaternion(1.0, 0.0, 0.0, 0.0);
    base.angle_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
    centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
    centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
    centroid.zmp_target  = Vector3(0.0, 0.0, 0.0);
    centroid.dcm_ref     = Vector3(0.0, 0.0, param.com_height);
    centroid.dcm_target  = Vector3(0.0, 0.0, param.com_height);
    foot[0].pos_ref = Vector3(0.0, -0.15/2.0, 0.0);
    foot[1].pos_ref = Vector3(0.0,  0.15/2.0, 0.0);
    foot[0].contact_ref = true;
    foot[1].contact_ref = true;
    
    //InitMarkers(io);

    //string path = cnoid::shareDir() + "/motion/perfume/aachan.bvh";
    //string path = cnoid::shareDir() + "/motion/cmu/02_05.bvh";
    //string path = cnoid::shareDir() + "/motion/cmu/02_06.bvh";
    //string path = cnoid::shareDir() + "/motion/cmu/13_07.bvh";
    string path = cnoid::shareDir() + "/motion/cmu/13_08.bvh";
    //string path = cnoid::shareDir() + "/motion/cmu/15_08.bvh";
    bvh.Load(path);
    baseName        = "hip";
    chestName       = "chest";
    headName        = "head";
    handName[0]     = "rHand";
    handName[1]     = "lHand";
    shoulderName[0] = "rShldr";
    shoulderName[1] = "lShldr";
    elbowName[0]    = "rForeArm";
    elbowName[1]    = "lForeArm";
    middleName[0]   = "rMid1";
    middleName[1]   = "lMid1";

    handScale  = 0.7;
    handOffset = Vector3(0.0, 0.0, -0.2);

    //bvh.Load("C:\\usr\\Develop\\choreonoid\\vnoid\\bvh\\aachan.bvh");
    //bvh.WriteCsv("bvh_motion.csv");

    baseNode  = bvh.FindNode(baseName );
    chestNode = bvh.FindNode(chestName);
    headNode  = bvh.FindNode(headName );
    for(int i = 0; i < 2; i++){
        handNode    [i] = bvh.FindNode(handName    [i]);
        shoulderNode[i] = bvh.FindNode(shoulderName[i]);
        elbowNode   [i] = bvh.FindNode(elbowName   [i]);
        middleNode  [i] = bvh.FindNode(middleName  [i]);
    }

    // init visualizer
    viz.header.numMaxFrames       = 1500;
    viz.header.numMaxLines        = 10;
    viz.header.numMaxSpheres      = 100;
    viz.header.numMaxBoxes        = 0;
    viz.header.numMaxCylinders    = 0;
    viz.header.numMaxLineVertices = 500;
    viz.Open();

}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // comp FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);
	
    double t = timer.time;
    bvh.CalcPose(t);

    // map base orientation
    base.pos_ref = baseNode->pos;
    base.ori_ref = baseNode->ori;

    // map chest joint angle
    Vector3 angle = ToRollPitchYaw(baseNode->ori.conjugate()*chestNode->ori);
    joint[0].q_ref = angle.z();  //< chest yaw
    joint[1].q_ref = angle.y();  //< chest pitch

    // map head joint angle
    angle = ToRollPitchYaw(chestNode->ori.conjugate()*headNode->ori);
    joint[2].q_ref = angle.z();
    joint[3].q_ref = angle.y();

    for(int i = 0; i < 2; i++){
        // calc desired elbow angle
        hand[i].fix_arm_twist = false;
        hand[i].fix_elbow_dir = true;

        Vector3 dir = (handNode[i]->pos - elbowNode[i]->pos).cross(elbowNode[i]->pos - shoulderNode[i]->pos);
        dir = dir/dir.norm();
        hand[i].elbow_dir = dir;

        // map hand pose
        hand[i].pos_ref = handScale*(handNode[i]->pos - baseNode->pos) + centroid.com_pos_ref + handOffset;

        Vector3 ydir = (i == 0 ? -1 : 1)*(handNode[i]->ori*Vector3::UnitZ());
        Vector3 zdir = -(middleNode[i]->pos - handNode[i]->pos);
        zdir = zdir/zdir.norm();
        Vector3 xdir = ydir.cross(zdir);
        zdir = xdir.cross(ydir);

        Matrix3 R;
        R.col(0) = xdir;
        R.col(1) = ydir;
        R.col(2) = zdir;

        hand[i].ori_ref = Quaternion(R);
    }
    
    /*
    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;
    */

    stabilizer.Update(timer, param, centroid, base, foot);

    // comp CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

    Robot::Actuate(timer, base, joint);

    Visualize();
    
	timer.Countup();
}

void MyRobot::Visualize(){
    VizInfo  info;
    info.iframe    = timer.time/0.025;
    info.ilines    = 0;
    info.isphere   = 0;
    info.ibox      = 0;
    info.icylinder = 0;
    Visualizer::Frame* fr = viz.data->GetFrame(info.iframe);
    fr->time = timer.time;

    // visualize bvh
    bvh.Visualize(&viz, info);

}

}
}
