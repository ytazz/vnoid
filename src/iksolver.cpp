#include "iksolver.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

void IkSolver::CompLegIk(const Vector3& pos, const Quaternion& ori, double l1, double l2, double* q){
    Vector3 angle = ToRollPitchYaw(ori);

    // hip yaw is directly determined from foot yaw
    q[0] = angle.z();

    // knee pitch from trigonometrics
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
        q[3] = pi - acos(tmp);
    }

    // ankle pos expressed in hip-yaw local
    Vector3 pos_local = AngleAxis(-q[0], Vector3::UnitZ())*pos;

    // hip roll
    q[1] = atan2(pos_local.y(), pos_local.z());

    // ankle pos expressed in hip yaw and hip roll local
    Vector3 pos_local2 = AngleAxis(-q[1], Vector3::UnitX())*pos_local;

    // hip pitch
    q[2] = atan2(pos_local2.x(), pos_local2.z()) - q[3];

    // ankle pitch and ankle roll are determined from foot pitch and foot roll
    q[4] = angle.y() - q[2] - q[3];
    q[5] = angle.x() - q[1];
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
        q[3] = pi - acos(tmp);
    }

    // shoulder pitch
    q[0] = atan2(pos.x(), pos.z());

    // wrist pos expressed in shoulder pitch local
    Vector3 pos_local = AngleAxis(-q[0], Vector3::UnitY())*pos;

    // shoulder roll
    q[1] = atan2(pos_local.y(), pos_local.z());

    // shoulder yaw is given
    q[2] = q2_ref;

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

}
}