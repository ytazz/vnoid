#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){

}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 43.0;
    param.nominal_inertia = Vector3(2.5, 2.5, 0.2);
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
    param.lower_leg_length = 0.3;

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
    param.leg_com[3] = Vector3(0.0, 0.0, -0.15);
    param.leg_com[4] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[5] = Vector3(0.0, 0.0,  0.0);

    //param.zmp_min = Vector3(-0.075, -0.025, -0.1);
    //param.zmp_max = Vector3( 0.075,  0.025,  0.1);
    //param.zmp_min = Vector3(-0.05, -0.02, -0.1);
    //param.zmp_max = Vector3( 0.05,  0.02,  0.1);
    //param.zmp_min = Vector3(-0.035, -0.015, -0.1);
    //param.zmp_max = Vector3( 0.035,  0.015,  0.1);
    param.zmp_min = Vector3(-0.0, -0.0, -0.1);
    param.zmp_max = Vector3( 0.0,  0.0,  0.1);
    
    param.Init();

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    // assign small PD gains to ankle joints so that torque-based ZMP control works well
    joint_pos_filter_cutoff = 10.0;
    joint.resize(30);
    joint[ 0].Set(500.0, 100.0, 100.0);
    joint[ 1].Set(500.0, 100.0, 100.0);
    joint[ 2].Set(500.0, 100.0, 100.0);
    joint[ 3].Set(500.0, 100.0, 100.0);
    joint[ 4].Set(500.0, 100.0, 100.0);
    joint[ 5].Set(500.0, 100.0, 100.0);
    joint[ 6].Set(500.0, 100.0, 100.0);
    joint[ 7].Set(500.0, 100.0, 100.0);
    joint[ 8].Set(500.0, 100.0, 100.0);
    joint[ 9].Set(500.0, 100.0, 100.0);
    joint[10].Set(500.0, 100.0, 100.0);
    joint[11].Set(500.0, 100.0, 100.0);
    joint[12].Set(500.0, 100.0, 100.0);
    joint[13].Set(500.0, 100.0, 100.0);
    joint[14].Set(500.0, 100.0, 100.0);
    joint[15].Set(500.0, 100.0, 100.0);
    joint[16].Set(500.0, 100.0, 100.0);
    joint[17].Set(500.0, 100.0, 100.0);
    joint[18].Set(2000.0, 400.0, 200.0);
    joint[19].Set(2000.0, 400.0, 200.0);
    joint[20].Set(2000.0, 400.0, 200.0);
    joint[21].Set(2000.0, 400.0, 200.0);
    joint[22].Set(100.0, 20.0, 100.0);
    joint[23].Set(100.0, 20.0, 100.0);
    joint[24].Set(2000.0, 400.0, 200.0);
    joint[25].Set(2000.0, 400.0, 200.0);
    joint[26].Set(2000.0, 400.0, 200.0);
    joint[27].Set(2000.0, 400.0, 200.0);
    joint[28].Set(100.0, 20.0, 100.0);
    joint[29].Set(100.0, 20.0, 100.0);
    
    // init hardware (simulator interface)
    gyro_filter_cutoff = 20.0;
	Robot::Init(io, timer, joint);

    // walking controller
    controller.min_contact_force = 100.0;
    controller.orientation_ctrl_gain_p = Vector3(400.0, 400.0, 10.0); //yaw 10.0
    controller.orientation_ctrl_gain_d = Vector3( 40.0,  40.0, 10.0); //yaw 10.0
    controller.orientation_ctrl_gain_i = Vector3(120.0, 120.0,  0.0);
    controller.orientation_ctrl_deadband = Vector3(0.0, 0.0, 0.0);
    controller.dcm_ctrl_gain       = 2.0;
    controller.nominal_duration    = 0.45;
    controller.min_duration        = 0.3;
    controller.max_dcm_distance    = 0.5;
    controller.dsp_rate            = 0.05;
    controller.descend_rate        = 0.0;
    controller.stride              = 0.0;
    controller.sway                = 0.0;
    controller.turn                = 0.0;
    controller.spacing             = 0.15;
    controller.swing_height        = 0.10;
    controller.stepping            = false;
    controller.base_tilt_rate      = 0.0;
    controller.base_tilt_damping_p = 1000.0;
    controller.base_tilt_damping_d = 100.0;
    controller.Ldmax = Vector3(30.0, 30.0, 0.0);

    base.ori_ref   = Quaternion(1.0, 0.0, 0.0, 0.0);
    base.angle_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
    centroid.com_vel_ref = Vector3(0.0, 0.0, 0.0);
    centroid.com_acc_ref = Vector3(0.0, 0.0, 0.0);
    centroid.zmp_ref     = Vector3(0.0, 0.0, 0.0);
    centroid.dcm_ref     = Vector3(0.0, 0.0, param.com_height);
    foot[0].pos_ref = Vector3(0.0, -0.10, 0.0);
    foot[1].pos_ref = Vector3(0.0,  0.10, 0.0);
    foot[0].contact_ref = true;
    foot[1].contact_ref = true;

    FILE* file = fopen("disturbance.csv", "r");
    fscanf(file, "%lf %lf %lf", &disturbance_magnitude, &disturbance_angle, &disturbance_duration);
    fclose(file);

    //InitMarkers(io);
}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // comp FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);
    /*
    if(timer.count > 0){
        centroid.com_vel = (centroid.com_pos - com_pos_prev)/timer.dt;
        centroid.dcm = centroid.com_pos + param.T*centroid.com_vel;
        com_pos_prev = centroid.com_pos;
    }
    */
    if(!controller.stepping && timer.time > 0.0){
        centroid.zmp_target   = foot[controller.sup].pos_ref;
        controller.lift_pos   = foot[controller.swg].pos_ref;
        controller.lift_angle = foot[controller.swg].angle_ref;
        controller.CalcDcmOffset(param);
        controller.duration = controller.nominal_duration;
        controller.t_land   = controller.nominal_duration + param.T*log(controller.dcm_offset[controller.sup].norm());
        //controller.dcm_scale = controller.dcm_offset[controller.sup].norm();
        controller.time_switch = timer.time - param.T*log(
            Vector2(centroid.dcm_ref.x() - foot[controller.sup].pos_ref.x(), centroid.dcm_ref.y() - foot[controller.sup].pos_ref.y()).norm()/controller.dcm_offset[controller.sup].norm()
        );
        controller.stepping    = true;
    }
    if(timer.time > 0.0){
        controller.Ldmax = Vector3(60.0, 60.0, 0);
    }
    /*
    double info[][2] = {
        {0.0,  0.0},
        {0.1,  0.0},
        {0.2,  0.0},
        {0.3,  0.0},
        {0.4,  0.0},
        {0.4,  0.0},
        {0.4,  0.0},
        {0.4,  0.0},
        {0.4,  0.0},
        {0.4,  0.0},
        {0.3,  0.0},
        {0.2,  0.0},
        {0.1, -0.0},
        {0.0, -0.0}
    };
    */
    /*
    double info[][2] = {
        {0.0,  0.0},
        {0.0,  0.0},
        {0.0,  0.0},
        {0.0,  0.0},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2,  3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4},
        {0.2, -3.14/4}
    };
    */
    double info[][2] = {
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},

        {0.05, 0.0},
        {0.1, 0.0},
        {0.15, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.20, 0.0},
        {0.15, 0.0},
        {0.1, 0.0},
        {0.05, 0.0},

        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        
        {0.0, 0.05},
        {0.0, 0.1},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.15},
        {0.0, 0.1},
        {0.0, 0.05}
    };
    /*
    */
    const int n = sizeof(info)/(sizeof(double)*2);
    controller.stride = info[controller.nstep%n][0];
    controller.turn   = info[controller.nstep%n][1];
    /*
    */
    //double s = std::min(std::max(0.0, 0.1*(timer.time - 1.0)), 1.0);
    //controller.stride  = s*0.2;
    //controller.turn    = s*0.1;
    //controller.turn    = (3.14/8)*sin(2*3.14*(1.0/3.0)*timer.time);
    //controller.turn    = (3.14/4)*(sin(2*3.14*(1.0/5.0)*timer.time) > 0.0 ? 1.0 : -1.0);
    //controller.sway    = s*0.0;
    //controller.spacing = (1-s)*0.15 + s*0.15;

    /*
    Link* target = io_body->link(0);
    static bool not_yet = true;
    if( 2.0 <= timer.time && timer.time <= 3.0 &&
        controller.sup == 0 &&
        controller.time_switch + controller.duration/2 - disturbance_duration/2 <= timer.time &&
        controller.time_switch + controller.duration/2 + disturbance_duration/2 >= timer.time
        //controller.time_switch <= timer.time &&
        //controller.time_switch + disturbance_duration >= timer.time
        ){
        // 500*0.05 = 25 Ns
        // 25/43 = 0.55 m/s approx 0.14 delta DCM
        //Vector3 f(500.0, 200.0, 0.0);
        //Vector3 f(-500.0, 0.0, 0.0);
        //Vector3 f(0.0, -400.0, 0.0);
        //Vector3 f(0.0, 300.0, 0.0);
        Vector3 f = (param.total_mass/(param.T*disturbance_duration))*disturbance_magnitude*Vector3(cos((3.1415/180.0)*disturbance_angle), sin((3.1415/180.0)*disturbance_angle), 0.0);
        target->f_ext  () = f;
	    target->tau_ext() = target->p().cross(f);
    }
    else{
        target->f_ext  () = Vector3::Zero();
	    target->tau_ext() = Vector3::Zero();
    }
    */
    controller.Update(timer, param, centroid, base, foot);

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;
    
    // comp CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

    Robot::Actuate(timer, base, joint);

    //UpdateMarkers(base, centroid, hand, foot);
    
	timer.Countup();
}

}
}
