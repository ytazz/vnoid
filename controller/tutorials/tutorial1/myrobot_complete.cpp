#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

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
    param.wrist_to_hand   [0] = Vector3(0.0,  0.0, -0.0);
    param.wrist_to_hand   [1] = Vector3(0.0,  0.0, -0.0);
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
    // assign small PD gains to ankle joints so that torque-based ZMP control works well
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
    
    /*
     * チュートリアル ステップ1
     * ロボットを空中に固定する
     */
    // ベースリンクの位置を直接指定するオプションを有効化
    base_actuation = true;

    // ベースリンクの座標を設定
    base.pos_ref = Vector3(0.0, 0.0, 1.0);

    Robot::Init(io, timer, joint);

    // スタビライザの設定
    stabilizer.orientation_ctrl_gain_p = 100.0;
    stabilizer.orientation_ctrl_gain_d = 10.0;
    stabilizer.dcm_ctrl_gain           = 2.0;
    stabilizer.base_tilt_rate          = 0.0;
    stabilizer.base_tilt_damping_p     = 10.0;
    stabilizer.base_tilt_damping_d     = 5.0;

    // ビジュアライザの設定
    visualizer.header.numMaxFrames       = 10000;
    visualizer.header.numMaxLines        = 0;
    visualizer.header.numMaxSpheres      = 10;
    visualizer.header.numMaxBoxes        = 10;
    visualizer.header.numMaxCylinders    = 0;
    visualizer.header.numMaxLineVertices = 0;
    visualizer.Open();
    
    centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
    centroid.dcm_ref     = Vector3(0.0, 0.0, param.com_height);
    centroid.dcm_target  = Vector3(0.0, 0.0, param.com_height);
    foot[0].pos_ref = Vector3(0.02, -0.15, 0.0);
    foot[1].pos_ref = Vector3(0.02,  0.15, 0.0);
    foot[0].contact_ref = true;
    foot[1].contact_ref = true;

    // ステッピングコントローラの設定
    stepping_controller.swing_height = 0.05;
    stepping_controller.swing_tilt   = 0.0;
    stepping_controller.dsp_duration = 0.05;

    // 数歩直進するfootstepの生成
    Step step;
	step.stride   = 0.0;
	step.turn     = 0.0;
	step.spacing  = 0.2;
	step.climb    = 0.0;
	step.duration = 0.5;
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	step.stride = 0.0;
	step.turn   = 0.0;
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
	footstep.steps.push_back(step);
		
	footstep_planner.Plan(param, footstep);
    footstep_planner.GenerateDCM(param, footstep);

    footstep_buffer.steps.push_back(footstep.steps[0]);
    footstep_buffer.steps.push_back(footstep.steps[1]);
}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // FK計算
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

    /*
     * チュートリアル ステップ2
     * ロボットに好きなポーズをとらせる 
     * 
     * Joint構造体はrobot_base.hを参照
     */
    // 腕を少し外側に広げる
    joint[ 5].q_ref = -0.2;
    joint[12].q_ref =  0.2;

    // 脚を少し折り曲げる
    joint[20].q_ref = -0.2;
    joint[21].q_ref =  0.4;
    joint[22].q_ref = -0.2;

    joint[26].q_ref = -0.2;
    joint[27].q_ref =  0.4;
    joint[28].q_ref = -0.2;
    /*
    // ゲーコン入力で腕を振る
    joystick.readCurrentState();
    joint[ 5].q_ref = -0.5 - 0.5*joystick.getPosition(Joystick::R_TRIGGER_AXIS);
    joint[12].q_ref =  0.5 + 0.5*joystick.getPosition(Joystick::L_TRIGGER_AXIS);
    */
    // 重心の目標位置
    //centroid.com_pos_ref = Vector3(0.0, 0.0, 1.0);
    //centroid.com_pos_ref = Vector3(0.0, 0.0, 1.0 - 0.2*joystick.getPosition(Joystick::R_TRIGGER_AXIS));
    
    // 足の目標位置と姿勢
    //foot[0].pos_ref = Vector3(0.0, -0.1, 0.4);
    //foot[1].pos_ref = Vector3(0.0,  0.1, 0.4);
    joystick.readCurrentState();

    while(footstep.steps.size() > 2)
        footstep.steps.pop_back();

    Step step;
    step.stride   = -0.2*joystick.getPosition(Joystick::L_STICK_V_AXIS);
    step.turn     = -0.5*joystick.getPosition(Joystick::L_STICK_H_AXIS);
    step.spacing  = 0.2;
	step.duration = 0.5;
	footstep.steps.push_back(step);
    footstep.steps.push_back(step);
    footstep.steps.push_back(step);
	
    footstep_planner.Plan(param, footstep);
    footstep_planner.GenerateDCM(param, footstep);
    
    // ステッピングコントローラの更新
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot);
    
    // スタビライザの更新
    stabilizer.Update(timer, param, centroid, base, foot);

    // IK計算
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint, false, true);
    
    // 可視化
    Visualize();
    /*
    */
    Robot::Actuate(timer, base, joint);

    timer.Countup();
}

void MyRobot::Visualize(){
    int iframe = (int)(timer.time/0.025);
    Visualizer::Frame* frame = visualizer.data->GetFrame(iframe);
    if(!frame)
        return;

    frame->time = timer.time;

    Visualizer::Sphere* sphere = visualizer.data->GetSphere(iframe, 0);
    sphere->pos    = centroid.com_pos_ref;
    sphere->radius = 0.02f;
    sphere->color  = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    sphere->alpha  = 0.5f;
}

}
}
