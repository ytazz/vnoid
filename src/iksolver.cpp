#include "iksolver.h"
#include "fksolver.h"
#include "rollpitchyaw.h"
#include "robot_base.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

void IkSolver::CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
    // hip pitch and knee pitch from trigonometrics
    double d = pos.norm();
    double c = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    double beta;
    
    //  singularity: too close
    if(c > 1.0){
        beta     = 0.0;
    }
    //  singularity: too far
    else if(c < -1.0){
        beta     = pi;
    }
    //  nonsingular
    else{
        beta  = acos(c);
    }

    q[3] = pi - beta;

    Quaternion qinv =   ori.conjugate();
    Vector3    phat = -(qinv*pos);
    Quaternion qhat =   qinv;

    // ankle pitch
    Vector3 phatd(-l1*sin(q[3]), 0.0, l1*cos(q[3]) + l2);
    double  c2 = phat.x()/sqrt(phatd.x()*phatd.x() + phatd.z()*phatd.z());
    double  gamma;
    if(c2 > 1.0){
        gamma = 0.0;
    }
    else if(c2 < -1.0){
        gamma = pi;
    }
    else{
        gamma = acos(c2);
    }

    double alpha = atan2(phatd.z(), phatd.x());
    q[4] = -alpha + gamma;
    
    // hip pos expressed in ankle pitch local
    Vector3 phatdd = AngleAxis(-q[4], Vector3::UnitY())*phatd;

    // ankle roll
    q[5] = -atan2(phat.z(), phat.y())
          + atan2(phatdd.z(), phatdd.y());
    if(q[5] >  pi) q[5] -= 2.0*pi;
    if(q[5] < -pi) q[5] += 2.0*pi;

    // desired hip rotation
    Quaternion qyy(AngleAxis(q[3] + q[4], Vector3::UnitY()));
    Quaternion qyyx   = qyy*AngleAxis(q[5], Vector3::UnitX());
    Quaternion qhip   = ori*qyyx.conjugate();
    Quaternion qzquad(AngleAxis(pi/2.0, Vector3::UnitZ()));

    // convert it to roll-pitch-yaw
    Vector3 angle_hip = ToRollPitchYaw(qzquad*qhip*qzquad.conjugate());

    // then wrist angles are determined
    q[0] =  angle_hip.z();
    q[1] =  angle_hip.y();
    q[2] = -angle_hip.x();
}

// note: the following IK code is wrong; you cannot determine hip yaw first. consider why!
void IkSolver::CompLegIkOld(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
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

void IkSolver::CompElbowAngle(const Vector3& pos, double l1, double l2, double* q){
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
}

void IkSolver::CompWristAngles(const Quaternion& ori, double* q){
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

void IkSolver::CompArmIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double q2_ref, double* q){
    // comp elbow angle
    CompElbowAngle(pos, l1, l2, q);

    // shoulder yaw is given
    q[2] = q2_ref;

    // wrist pos expressed in shoulder pitch-roll local
    Vector3 pos_local = AngleAxis(q[2], Vector3::UnitZ())*Vector3(-l2*sin(q[3]), 0.0, -l1 - l2*cos(q[3]));

    // shoulder roll
    double tmp = pos.y()/sqrt(pos_local.y()*pos_local.y() + pos_local.z()*pos_local.z());
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

    // comp wrist angles
    CompWristAngles(ori, q);
}

void IkSolver::CompArmIk2(const Vector3& pos, const Quaternion& ori, double l1, double l2, const Vector3& elbow_y, double* q){
    // comp elbow angle
    CompElbowAngle(pos, l1, l2, q);

    // wrist pos expressed in shoulder pitch-roll-yaw local
    Vector3 pos_local = Vector3(-l2*sin(q[3]), 0.0, -l1 - l2*cos(q[3]));

    // calc rotation that brings pos_local to pos
    Quaternion q1;
    q1.setFromTwoVectors(pos_local, pos);

    // calc extra rotation around the line penetrating the shoulder and the wrist
    // to bring the elbow axis to the desired direction
    Vector3 axis = pos_local/pos_local.norm();
    Vector3 dir  = q1.conjugate()*elbow_y;

    double theta = atan2(-dir.x()/axis.z(), dir.y());
    AngleAxis q2(theta, axis);

    // determine shoulder 3-axis rotations
    // convert it to roll-pitch-yaw
    Quaternion qzquad(AngleAxis(pi/2.0, Vector3::UnitZ()));
    Vector3 angle_shoulder = ToRollPitchYaw(qzquad*(q1*q2).conjugate()*qzquad.conjugate());

    // then wrist angles are determined
    q[0] =  angle_shoulder.x();
    q[1] = -angle_shoulder.y();
    q[2] = -angle_shoulder.z();

    // comp wrist angles
    CompWristAngles(ori, q);
}

void IkSolver::Comp(const Param& param, const Base& base, const vector<Hand>& hand, const vector<Foot>& foot, vector<Joint>& joint){
    Vector3    pos_local;
    Quaternion ori_local;
    double     q[7];

    // comp arm ik
    for(int i = 0; i < 2; i++){
        pos_local = base.ori_ref.conjugate()*(hand[i].pos_ref - hand[i].ori_ref*param.wrist_to_hand[i] - base.pos_ref) - param.base_to_shoulder[i];
        ori_local = base.ori_ref.conjugate()* hand[i].ori_ref;

        CompArmIk(pos_local, ori_local, param.upper_arm_length, param.lower_arm_length, hand[i].arm_twist, q);
        
        for(int j = 0; j < 7; j++){
            joint[param.arm_joint_index[i] + j].q_ref = q[j];
        }
    }
    
    // comp leg ik
    for(int i = 0; i < 2; i++){
        pos_local = base.ori_ref.conjugate()*(foot[i].pos_ref - foot[i].ori_ref*param.ankle_to_foot[i] - base.pos_ref) - param.base_to_hip[i];
        ori_local = base.ori_ref.conjugate()* foot[i].ori_ref;

        CompLegIk(pos_local, ori_local, param.upper_leg_length, param.lower_leg_length, q);

        for(int j = 0; j < 6; j++){
            joint[param.leg_joint_index[i] + j].q_ref = q[j];
        }
    }
}

void IkSolver::Comp(FkSolver* fk_solver, const Param& param, Centroid& centroid, Base& base, vector<Hand>& hand, vector<Foot>& foot, vector<Joint>& joint){
    // objects to store temprary FK results
    joint_tmp.resize(joint.size());
    hand_tmp.resize(hand.size());
    foot_tmp.resize(foot.size());

    // initialize base position with desired CoM position
    base.pos_ref = centroid.com_pos_ref;

    const double eps = 1.0e-5;
    
    // maximum loop count is limited
    int cnt = 0;
    while(cnt++ < 10){
        // comp ik
        Comp(param, base, hand, foot, joint);

        // copy q_ref to q
        for(int i = 0; i < joint.size(); i++)
            joint_tmp[i].q = joint[i].q_ref;

        // copy base link pose
        base_tmp.pos = base.pos_ref;
        base_tmp.ori = base.ori_ref;

        // comp fk
        fk_solver->Comp(param, joint_tmp, base_tmp, centroid_tmp, hand_tmp, foot_tmp);

        // calc difference of desired and calculated CoM position
        Vector3 diff = centroid.com_pos_ref - centroid_tmp.com_pos;

        // break if error is small enough
        if(diff.norm() < eps)
            break;

        // otherwise, adjust base link position
        base.pos_ref += diff;
    }

    // calc leg joint torque
    Eigen::Matrix<double,6,6> J;
    Eigen::Matrix<double,6,1> F;
    Eigen::Matrix<double,6,1> tau;
    double q[6];

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            q[j] = joint[param.leg_joint_index[i] + j].q_ref;
        }
        fk_solver->CompLegJacobian(param.upper_leg_length, param.lower_leg_length, q, J);

        Quaternion ori_local = base.ori_ref.conjugate()* foot[i].ori_ref;
        Vector3 fref = ori_local*foot[i].force_ref;
        Vector3 mref = ori_local*foot[i].moment_ref;
        F[0] = fref.x();
        F[1] = fref.y();
        F[2] = fref.z();
        F[3] = mref.x();
        F[4] = mref.y();
        F[5] = mref.z();

        tau = J.transpose()*F;
        //printf("F  : %f %f %f %f %f %f\n", F[0], F[1], F[2], F[3], F[4], F[5]);
        //printf("tau: %f %f %f %f %f %f\n", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);

        for(int j = 0; j < 6; j++){
            joint[param.leg_joint_index[i] + j].u_ref = -tau[j];
        }
    }
}

}
}