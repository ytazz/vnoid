#include "robot_demo.h"

using namespace std;

namespace cnoid{
namespace vnoid{

RobotDemo::RobotDemo(){

}

void RobotDemo::InitMarkers(SimpleControllerIO* io){
    // configure marker links for visualization
    marker_index = 31;
    num_markers  = 18;
    for (int i = 0; i < num_markers; i++) {
        io_body->link(marker_index + i)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(marker_index + i));
    }
}

void RobotDemo::UpdateMarkers(const Base& base, const Centroid& centroid, const vector<Hand>& hand, const vector<Foot>& foot){
    // relative pose of desired and actual poses of base link
    Vector3    p_sim = io_body->link(0)->p();
    Quaternion q_sim(io_body->link(0)->R());

    Vector3    p_fk = base.pos;
    Quaternion q_fk = base.ori;

    Vector3    p_ik = base.pos_ref;
    Quaternion q_ik = base.ori_ref;

    Quaternion q_fk_rel = q_sim*q_fk.conjugate();
    Vector3    p_fk_rel = p_sim - q_fk_rel*p_fk;

    Quaternion q_ik_rel = q_sim*q_ik.conjugate();
    Vector3    p_ik_rel = p_sim - q_ik_rel*p_ik;

    // update marker poses for visualization
    io_body->link(marker_index +  0)->p() =  q_fk_rel*hand[0].pos + p_fk_rel;
    io_body->link(marker_index +  0)->R() = (q_fk_rel*hand[0].ori).matrix();
    io_body->link(marker_index +  1)->p() =  q_fk_rel*hand[1].pos + p_fk_rel;
    io_body->link(marker_index +  1)->R() = (q_fk_rel*hand[1].ori).matrix();
    io_body->link(marker_index +  2)->p() =  q_fk_rel*foot[0].pos + p_fk_rel;
    io_body->link(marker_index +  2)->R() = (q_fk_rel*foot[0].ori).matrix();
    io_body->link(marker_index +  3)->p() =  q_fk_rel*foot[1].pos + p_fk_rel;
    io_body->link(marker_index +  3)->R() = (q_fk_rel*foot[1].ori).matrix();
    io_body->link(marker_index +  4)->p() =  q_ik_rel*hand[0].pos_ref + p_ik_rel;
    io_body->link(marker_index +  4)->R() = (q_ik_rel*hand[0].ori_ref).matrix();
    io_body->link(marker_index +  5)->p() =  q_ik_rel*hand[1].pos_ref + p_ik_rel;
    io_body->link(marker_index +  5)->R() = (q_ik_rel*hand[1].ori_ref).matrix();
    io_body->link(marker_index +  6)->p() =  q_ik_rel*foot[0].pos_ref + p_ik_rel;
    io_body->link(marker_index +  6)->R() = (q_ik_rel*foot[0].ori_ref).matrix();
    io_body->link(marker_index +  7)->p() =  q_ik_rel*foot[1].pos_ref + p_ik_rel;
    io_body->link(marker_index +  7)->R() = (q_ik_rel*foot[1].ori_ref).matrix();
	io_body->link(marker_index +  8)->p() =  q_fk_rel*centroid.com_pos     + p_fk_rel;
	io_body->link(marker_index +  9)->p() =  q_ik_rel*centroid.com_pos_ref + p_ik_rel;
	io_body->link(marker_index + 10)->p() =  q_ik_rel*centroid.zmp     + p_ik_rel;
	io_body->link(marker_index + 11)->p() =  q_ik_rel*centroid.zmp_ref + p_ik_rel;
	io_body->link(marker_index + 12)->p() =  q_ik_rel*centroid.dcm_ref + p_ik_rel;

    // move away others
    Vector3 farbelow(0.0, 0.0, -100.0);
	io_body->link(marker_index + 13)->p() = farbelow;
	io_body->link(marker_index + 14)->p() = farbelow;
	io_body->link(marker_index + 15)->p() = farbelow;
    io_body->link(marker_index + 16)->p() = farbelow;
	io_body->link(marker_index + 17)->p() = farbelow;
}

void RobotDemo::UpdateMarkers(const Base& base, const Footstep& footstep_buffer){
    // relative pose of desired and actual poses of base link
    Vector3    p = io_body->link(0)->p();
    Quaternion q(io_body->link(0)->R());

    Vector3    pref = base.pos_ref;
    Quaternion qref = base.ori_ref;

    Quaternion qrel = q*qref.conjugate();
    Vector3    prel = p - qrel*pref;

    const Step& stb0 = footstep_buffer.steps[0];
    const Step& stb1 = footstep_buffer.steps[1];
    int sup = stb0.side;
    int swg = !sup;

    Vector3    psup = stb0.foot_pos[sup];
    Quaternion qsup = stb0.foot_ori[sup];

    Vector3    pland = stb1.foot_pos[swg];
    Quaternion qland = stb1.foot_ori[swg];

    // update marker poses for visualization
	io_body->link(marker_index + 13)->p() =  qrel*stb0.zmp + prel;
	io_body->link(marker_index + 14)->p() =  qrel*stb0.dcm + prel;
	io_body->link(marker_index + 15)->p() =  qrel*stb1.dcm + prel;
    io_body->link(marker_index + 16)->p() =  qrel*psup + prel;
	io_body->link(marker_index + 16)->R() = (qrel*qsup).matrix();
    io_body->link(marker_index + 17)->p() =  qrel*pland + prel;
	io_body->link(marker_index + 17)->R() = (qrel*qland).matrix();
}

}
}
