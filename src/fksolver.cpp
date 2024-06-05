#include "fksolver.h"
#include "rollpitchyaw.h"
#include "robot_base.h"

namespace cnoid{
namespace vnoid{

void FkSolver::CompLegFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis){
    Vector3  pos_local[6];
    Vector3  axis_local[6];
    pos_local[0] = Vector3(0.0, 0.0, 0.0);
    pos_local[1] = Vector3(0.0, 0.0, 0.0);
    pos_local[2] = Vector3(0.0, 0.0, 0.0);
    pos_local[3] = Vector3(0.0, 0.0, -l1);
    pos_local[4] = Vector3(0.0, 0.0, -l2);
    pos_local[5] = Vector3(0.0, 0.0, 0.0);
    axis_local[0] = Vector3::UnitZ();
    axis_local[1] = Vector3::UnitX();
    axis_local[2] = Vector3::UnitY();
    axis_local[3] = Vector3::UnitY();
    axis_local[4] = Vector3::UnitY();
    axis_local[5] = Vector3::UnitX();

    Vector3    pbase(0.0, 0.0, 0.0);
    Quaternion qbase(1.0, 0.0, 0.0, 0.0);
    for(int i = 0; i < 6; i++){
        axis[i] = (i == 0 ? qbase : ori[i-1])*axis_local[i];
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*pos_local[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*AngleAxis(q[i], axis_local[i]);
    }
}

void FkSolver::CompLegFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori){
    Vector3    _pos[6];
    Quaternion _ori[6];
    Vector3    _axis[6];

    CompLegFk(l1, l2, q, _pos, _ori, _axis);

    pos = _pos[5];
    ori = _ori[5];
}

void FkSolver::CompLegJacobian(double l1, double l2, const double* q, Eigen::Matrix<double,6,6>& J){
    Vector3    _pos[6];
    Quaternion _ori[6];
    Vector3    _axis[6];

    CompLegFk(l1, l2, q, _pos, _ori, _axis);

    for(int j = 0; j < 6; j++){
        Vector3 w = _axis[j];
        Vector3 v = _axis[j].cross(_pos[5] - _pos[j]);
        
        J(0,j) = v[0];
        J(1,j) = v[1];
        J(2,j) = v[2];
        J(3,j) = w[0];
        J(4,j) = w[1];
        J(5,j) = w[2];
    }
}

void FkSolver::CompArmFk(double l1, double l2, const double* q, Vector3* pos, Quaternion* ori, Vector3* axis){
    Vector3  pos_local[7];
    Vector3  axis_local[7];
    pos_local[0] = Vector3(0.0, 0.0, 0.0);
    pos_local[1] = Vector3(0.0, 0.0, 0.0);
    pos_local[2] = Vector3(0.0, 0.0, 0.0);
    pos_local[3] = Vector3(0.0, 0.0, -l1);
    pos_local[4] = Vector3(0.0, 0.0, -l2);
    pos_local[5] = Vector3(0.0, 0.0, 0.0);
    pos_local[6] = Vector3(0.0, 0.0, 0.0);
    axis_local[0] = Vector3::UnitY();
    axis_local[1] = Vector3::UnitX();
    axis_local[2] = Vector3::UnitZ();
    axis_local[3] = Vector3::UnitY();
    axis_local[4] = Vector3::UnitZ();
    axis_local[5] = Vector3::UnitY();
    axis_local[6] = Vector3::UnitX();

    Vector3    pbase(0.0, 0.0, 0.0);
    Quaternion qbase(1.0, 0.0, 0.0, 0.0);
    for(int i = 0; i < 7; i++){
        axis[i] = (i == 0 ? qbase : ori[i-1])*axis_local[i];
        pos [i] = (i == 0 ? pbase : pos[i-1]) + (i == 0 ? qbase : ori[i-1])*pos_local[i];
        ori [i] = (i == 0 ? qbase : ori[i-1])*AngleAxis(q[i], axis_local[i]);
    }
}

void FkSolver::CompArmFk(double l1, double l2, const double* q, Vector3& pos, Quaternion& ori){
    Vector3    _pos[7];
    Quaternion _ori[7];
    Vector3    _axis[7];

    CompArmFk(l1, l2, q, _pos, _ori, _axis);

    pos = _pos[6];
    ori = _ori[6];
}

void FkSolver::Comp(const Param& param, const vector<Joint>& joint, const Base& base, Centroid& centroid, vector<Hand>& hand, vector<Foot>& foot){
    Vector3     arm_pos [2][7];
    Quaternion  arm_ori [2][7];
    Vector3     arm_axis[2][7];
    Vector3     leg_pos [2][6];
    Quaternion  leg_ori [2][6];
    Vector3     leg_axis[2][6];
    double      q[7];

    // comp arm fk
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 7; j++){
            q[j] = joint[param.arm_joint_index[i] + j].q;
        }

        CompArmFk(param.upper_arm_length, param.lower_arm_length, q, &arm_pos[i][0], &arm_ori[i][0], &arm_axis[i][0]);

        hand[i].ori   = base.ori*arm_ori[i][6];
        hand[i].angle = ToRollPitchYaw(hand[i].ori);
        hand[i].pos   = base.pos + base.ori*(arm_pos[i][6] + param.base_to_shoulder[i]) + hand[i].ori*param.wrist_to_hand[i];
    }

    // comp leg fk
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            q[j] = joint[param.leg_joint_index[i] + j].q;
        }

        CompLegFk(param.upper_leg_length, param.lower_leg_length, q, &leg_pos[i][0], &leg_ori[i][0], &arm_axis[i][0]);

        foot[i].ori   = base.ori*leg_ori[i][5];
        foot[i].angle = ToRollPitchYaw(foot[i].ori);
        foot[i].pos   = base.pos + base.ori*(leg_pos[i][5] + param.base_to_hip[i]) + foot[i].ori*param.ankle_to_foot[i];
    }

    // comp center_of_mass
    double total_mass = 0.0;
    Vector3  com(0.0, 0.0, 0.0);

    total_mass += param.trunk_mass;
    com += param.trunk_mass * param.trunk_com;

    for(int i = 0; i < 2; i++){
        for (int j = 0; j < 7; j++) {
            total_mass += param.arm_mass[j];
            com += param.arm_mass[j] * (param.base_to_shoulder[i] + arm_pos[i][j] + arm_ori[i][j] * param.arm_com[j]);
        }
    }
    for(int i = 0; i < 2; i++){
        for (int j = 0; j < 6; j++) {
            total_mass += param.leg_mass[j];
            com += param.leg_mass[j] * (param.base_to_hip[i] + leg_pos[i][j] + leg_ori[i][j] * param.leg_com[j]);
        }
    }

    com /= total_mass;

    centroid.com_pos = base.pos + base.ori * com;
}

}
}