#include "iksolver.h"
#include "rollpitchyaw.h"
#include "robot.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

void IkSolver::CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
    Vector3 angle = ToRollPitchYaw(ori);

    // hip yaw is directly determined from foot yaw
    q[0] = angle.z();

    // ankle pos expressed in hip-yaw local
    Vector3 pos_local = AngleAxis(-q[0], Vector3::UnitZ())*pos;

    // hip roll
    q[1] = atan2(pos_local.y(), -pos_local.z());

    // ankle pos expressed in hip yaw and hip roll local
    Vector3 pos_local2 = AngleAxis(-q[1], Vector3::UnitX())*pos_local;

    double  alpha = -atan2(pos_local2.x(), -pos_local2.z());
    
    // hip pitch and knee pitch from trigonometrics
    double d   = pos.norm();
    double tmp = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    //  singularity: too close
    if(tmp > 1.0){
        q[3] = pi;
        q[2] = alpha;
    }
    //  singularity: too far
    else if(tmp < -1.0){
        q[3] = 0.0;
        q[2] = alpha;
    }
    //  nonsingular
    else{
        q[3] = pi - acos(tmp);
        q[2] = alpha - asin((l2/d)*sin(q[3]));
    }

    Quaternion qzxyyy = AngleAxis(q[0],      Vector3::UnitZ())
                       *AngleAxis(q[1],      Vector3::UnitX())
                       *AngleAxis(q[2]+q[3], Vector3::UnitY());
    Quaternion qrel = qzxyyy.conjugate()*ori;
    Vector3 angle_rel = ToRollPitchYaw(qrel);

    q[4] = angle_rel.y();
    q[5] = angle_rel.x();

    /*
    // easy way, but not correct

    // ankle pitch
    q[4] = angle.y() - q[2] - q[3];

    // ankle roll
    q[5] = angle.x() - q[1];
    */
    
}

void IkSolver::CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q){
    // elbow pitch from trigonometrics
    double tmp = (l1*l1 + l2*l2 - pos.squaredNorm())/(2*l1*l2);
    //  singularity: too close
    if(tmp > 1.0){
        q[3] = pi;
    }
    //  singularity: too far
    else if(tmp < -1.0){
        q[3] = 0.0;
    }
    //  nonsingular
    else{
        q[3] = -(pi - acos(tmp));
    }

    // shoulder yaw is given
    q[2] = q2_ref;

    // wrist pos expressed in shoulder pitch-roll local
    Vector3 pos_local = AngleAxis(q[2], Vector3::UnitZ())*Vector3(-l2*sin(q[3]), 0.0, -l1 - l2*cos(q[3]));

    // shoulder roll
    tmp = pos.y()/sqrt(pos_local.y()*pos_local.y() + pos_local.z()*pos_local.z());
    double delta;
    if(tmp > 1.0){
        delta = 0.0;
    }
    else if(tmp < -1.0){
        delta = pi;
    }
    else{
        delta =  acos(tmp);
    }
    
    double alpha = atan2(pos_local.z(), pos_local.y());
    q[1] = -alpha - delta;
    
    // wrist pos expressed in shoulder pitch local
    Vector3 pos_local2 = AngleAxis(q[1], Vector3::UnitX())*pos_local;

    // shoulder pitch
    q[0] = atan2(pos.x(), pos.z())
         - atan2(pos_local2.x(), pos_local2.z());
    if(q[0] >  pi) q[0] -= 2.0*pi;
    if(q[0] < -pi) q[0] += 2.0*pi;

    // desired hand orientation
    Quaternion qhand = ( AngleAxis(q[0], Vector3::UnitY())
                        *AngleAxis(q[1], Vector3::UnitX())
                        *AngleAxis(q[2], Vector3::UnitZ())
                        *AngleAxis(q[3], Vector3::UnitY()) ).conjugate()*ori;

    // convert it to roll-pitch-yaw
    Vector3 angle_hand = ToRollPitchYaw(qhand);

    // then wrist angles are determined
    q[4] = angle_hand.z();
    q[5] = angle_hand.y();
    q[6] = angle_hand.x();

}

void IkSolver::Comp(const Param& param, const Centroid& centroid, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint){
    
    Vector3    pos_local;
    Quaternion ori_local;
    double     q[7];

    // comp arm ik
    for(int i = 0; i < 2; i++){
        pos_local = base.ori_ref.conjugate()*(hand[i].pos_ref - hand[i].ori_ref*param.wrist_to_hand[i] - centroid.com_pos_ref) - param.base_to_shoulder[i];
        ori_local = base.ori_ref.conjugate()* hand[i].ori_ref;

        CompArmIk(pos_local, ori_local, param.upper_arm_length, param.lower_arm_length, hand[i].arm_twist, q);
        
        for(int j = 0; j < 7; j++){
            joint[param.arm_joint_index[i] + j].q_ref = q[j];
        }
    }
    printf("\n");

    // comp leg ik
    for(int i = 0; i < 2; i++){
        pos_local = base.ori_ref.conjugate()*(foot[i].pos_ref - foot[i].ori_ref*param.ankle_to_foot[i] - centroid.com_pos_ref) - param.base_to_hip[i];
        ori_local = base.ori_ref.conjugate()* foot[i].ori_ref;

        CompLegIk(pos_local, ori_local, param.upper_leg_length, param.lower_leg_length, q);

        for(int j = 0; j < 6; j++){
            joint[param.leg_joint_index[i] + j].q_ref = q[j];
        }
    }
}

}
}