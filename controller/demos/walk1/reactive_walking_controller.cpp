#include "reactive_walking_controller.h"

#include "robot.h"
#include "rollpitchyaw.h"

#include <Eigen/Geometry>

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

ReactiveWalkingController::ReactiveWalkingController(){
    swing_height        = 0.05;
    swing_tilt          = 0.0;
    nominal_duration    = 0.4;
    min_duration        = 0.1;
    max_dcm_distance    = 0.3;
    dsp_rate            = 0.2;
    descend_rate        = 0.0;
    descend_depth       = 0.0;

    stride  = 0.0;
    sway    = 0.0;
    spacing = 0.0;
    turn    = 0.0;
    
    stepping = false;
    sup      = 0;
    swg      = 1;
    nstep    = 0;

    dcm_offset[0] = Vector2::Zero();
    dcm_offset[1] = Vector2::Zero();
    lift_pos      = Vector3::Zero();
    lift_angle    = Vector3::Zero();
    land_pos      = Vector3::Zero();
    land_angle    = Vector3::Zero();
    land_dcm      = Vector3::Zero();
    angle_switch  = Vector3::Zero();
    theta_i       = Vector3::Zero();

    yaw_angle_des  = 0.0;
    yaw_angvel_des = 0.0;
    theta_ref = Vector3::Zero();
    omega_ref = Vector3::Zero();

    fileLogFk        = 0;
    fileLogIk        = 0;
    fileLogCtrl      = 0;
    fileLogStepFk[0] = 0;
    fileLogStepFk[1] = 0;
    fileLogStepIk[0] = 0;
    fileLogStepIk[1] = 0;

    enable_logging = false;
    enable_logging = false;
}

void ReactiveWalkingController::CalcDcmOffset(const Param& param){
    double tau   = nominal_duration;
    double T     = param.T;
    double alpha = exp(tau/T);

    Vector2 p1, p2;
    auto Rhalf = Matrix2(Eigen::Rotation2D<double>(turn/2));
    auto R     = Matrix2(Eigen::Rotation2D<double>(turn  ));
    if(turn == 0.0){
        p1 = Vector2(stride/2, sway/2 + spacing);
        p2 = Vector2(stride  , sway            );
    }
    else{
        double r = (stride/2)/(turn/2);
        p1 =             Vector2(0.0, r + spacing/2) + Rhalf*Vector2(0.0, -r + spacing/2 + sway/2);
        p2 = p1 + Rhalf*(Vector2(0.0, r - spacing/2) + Rhalf*Vector2(0.0, -r - spacing/2 + sway/2));
    }
    dcm_offset[0] = ((alpha*alpha)*Matrix2::Identity() - R).inverse()*((alpha - 1.0)*p1 + p2);
    dcm_offset[1] = Rhalf.transpose()*(alpha*dcm_offset[0] - p1);
}

inline Matrix3 cross_mat(const Vector3& r){
    Matrix3 m;
    m <<  0   , -r(2),  r(1),
          r(2),  0   , -r(0),
         -r(1),  r(0),  0;
    return m;
}

double ReactiveWalkingController::CalcLanding(const Param& param, const Base& base, const Centroid& centroid, const vector<Foot>& foot, double t_land_adjust){
    double  T = param.T;
    Vector3 offset(0.0, 0.0, param.com_height);
    
    // time to landing
    ttl = t_land_adjust - t_now;

	// predict dcm at landing
    Vector3 zmp_mod = centroid.zmp_target;// - 0.0*(T*T)*delta;
    land_dcm = (zmp_mod + offset) + exp(ttl/T)*(centroid.dcm_ref - (zmp_mod + offset));

    // dcm offset adjustment based on base rotation error
    double m = param.total_mass;
	double h = param.com_height;
	dcm_offset_mod.x() = -1.0*((1/(h*m))*param.nominal_inertia.y())*(Kp.y()*theta.y() + (Kd.y()+T*Kp.y())*omega.y())/(1/(T*T) + Kd.y()/T + Kp.y());
    dcm_offset_mod.y() =  1.0*((1/(h*m))*param.nominal_inertia.x())*(Kp.x()*theta.x() + (Kd.x()+T*Kp.x())*omega.x())/(1/(T*T) + Kd.x()/T + Kp.x());

    // landing adjustment based on dcm offset
    land_angle = foot[sup].angle_ref + Vector3(0.0, 0.0, yaw_angvel_des*duration);
    land_pos   = (land_dcm - offset) 
        - AngleAxis(base.angle_ref.z(), Vector3::UnitZ())*Vector3(dcm_offset_mod.x(), dcm_offset_mod.y(), 0.0)
        - AngleAxis(land_angle.z(), Vector3::UnitZ())*Vector3(dcm_offset[swg].x(), dcm_offset[swg].y(), 0.0);

    // project land pos to avoid self-collision
    Vector3 land_pos_local = foot[sup].ori_ref.conjugate()*(land_pos - foot[sup].pos_ref);
    Vector3 land_pos_local_proj = land_pos_local;
    land_pos = foot[sup].pos_ref + foot[sup].ori_ref*land_pos_local_proj;

    double cost = 10*(land_pos_local - land_pos_local_proj).norm() + std::abs(t_land - t_land_adjust);
    return cost;
}

void ReactiveWalkingController::Update(const Timer& timer, const Param& param, Centroid& centroid, Base& base, vector<Foot>& foot){
    CalcZmp(param, centroid, foot);

    // calc dcm and zmp 
	double T = param.T;
	double m = param.total_mass;
	double h = param.com_height;
	double T_mh  = T/(m*h);
	double T2_mh = T*T_mh;
    Kp = orientation_ctrl_gain_p;
    Kd = orientation_ctrl_gain_d;
    Ki = orientation_ctrl_gain_i;
    I = param.nominal_inertia;
    Ld_local, theta, omega, omegadd;
	Vector3 offset(0.0, 0.0, param.com_height);
    Vector3 r  = centroid.com_pos_ref - centroid.zmp_target;
    
    // error between desired and actual base link orientation
    theta[2] = base.angle[2] - base.angle_ref[2];
    omega[2] = base.angvel[2] - base.angvel_ref[2];
    theta_i[2] += theta[2]*timer.dt;
    
    // desired angular acceleration for regulating orientation (in local coordinate)
	omegadd[2] = -(Kp[2]*theta[2] + Kd[2]*omega[2] + Ki[2]*theta_i[2]);
    Ld_local[2] = I[2]*omegadd[2];
    Ld_local[2] = std::min(std::max(-Ldmax[2], Ld_local[2]), Ldmax[2]);
    
    double rx2 = r.x()*r.x();
    double ry2 = r.y()*r.y();
    
    if(rx2 + ry2 > 1.0e-6){
        delta2.x() =  (r.y()/((rx2 + ry2)*m))*Ld_local.z();
        delta2.y() = -(r.x()/((rx2 + ry2)*m))*Ld_local.z();
        delta2.z() =  0.0;
        
        Vector3 Ld2;
        Ld2.x() =  m*r.z()*delta2.y();
        Ld2.y() = -m*r.z()*delta2.x();
        Ld2.z() =  0.0;
        Vector3 Ld2_local = Eigen::AngleAxis(-base.angle_ref.z(), Vector3::UnitZ())*Ld2;
        
        Vector3 omegadd2;
        omegadd2.x() = Ld2_local.x()/I.x() - 50.0*base.angle_ref.x() - 10.0*base.angvel_ref.x();
        omegadd2.y() = Ld2_local.y()/I.y() - 50.0*base.angle_ref.y() - 10.0*base.angvel_ref.y();
        omegadd2.z() = 0.0;

        base.angle_ref  += base.angvel_ref*timer.dt;
        base.angvel_ref += omegadd2*timer.dt;
    }

    // error between desired and actual base link orientation
    for(int i = 0; i < 2; i++){
        theta[i] = base.angle[i]  - base.angle_ref[i];
        omega[i] = base.angvel[i] - base.angvel_ref[i];
        theta_i[i] += theta[i]*timer.dt;
    }
    
    // desired angular acceleration for regulating orientation (in local coordinate)
	for(int i = 0; i < 2; i++){
        omegadd[i] = -(Kp[i]*theta[i] + Kd[i]*omega[i] + Ki[i]*theta_i[i]);
        Ld_local[i] = I[i]*omegadd[i];
        Ld_local[i] = std::min(std::max(-Ldmax[i], Ld_local[i]), Ldmax[i]);
    }
    
    // desired moment (in global coordinate);
	Vector3 Ld = AngleAxis(base.angle_ref.z(), Vector3::UnitZ()) * Ld_local;
    
    // virtual disturbance applied to DCM dynamics to generate desired recovery moment
    delta  = Vector3::Zero();
    double tau_z = 0.0;
    delta.x() = -(1/(m*h))*Ld.y();
    delta.y() =  (1/(m*h))*Ld.x();
    
    if(stepping){
	    // calc desired DCM offset from gait specs
        CalcDcmOffset(param);

        // calc time from dcm
        Vector2 xi(centroid.dcm_ref.x() - centroid.zmp_target.x(), centroid.dcm_ref.y() - centroid.zmp_target.y());
        t_now = T*log(xi.norm());
    
        // adjust ttl so that predicted landing position is inside reachable region
        double t_land_adjust = t_land;
        double t_land_best   = t_land;
        double cost_min = 1.0e10;
        while(t_land_adjust >= t_now){
            double cost = CalcLanding(param, base, centroid, foot, t_land_adjust);

            if(cost < cost_min){
                cost_min = cost;
                t_land_best = t_land_adjust;
            }
            
            t_land_adjust -= 0.01;
        }
        CalcLanding(param, base, centroid, foot, t_land_best);

        if(ttl <= 0.0 || (ttl < 0.1 && foot[swg].contact)){
            printf("switch\n");

            // log footstep
            if(enable_logging){
                SaveFootstep(param, foot, swg, true );
                SaveFootstep(param, foot, swg, false);
            }

            // determine next support foot
            sup = !sup;
            swg = !sup;
    
            time_switch = timer.time;
            nstep++;

            Vector2 xi(centroid.dcm_ref.x() - foot[sup].pos_ref.x(), centroid.dcm_ref.y() - foot[sup].pos_ref.y());
            
            // current DCM-time
            t_now  = T*log(xi.norm());

            // nominal DCM-time of next landing
            t_land = nominal_duration + T*log(dcm_offset[sup].norm());

            if(t_land - t_now < min_duration){
                t_land = t_now + min_duration;
            }
            
            duration = t_land - t_now;

            // set next desired yaw angular velocity
            yaw_angle_des  += turn/2;
            yaw_angvel_des = (turn/2)/duration + 0.0*(yaw_angle_des - base.angle.z())/duration;
            
            // reset height of both feet
            for(int i = 0; i < 2; i++)
                foot[i].pos_ref.z() = 0.0;

            // store base angle
            angle_switch = base.angle_ref;

            //
            lift_pos   = Vector3(foot[swg].pos_ref.x(), foot[swg].pos_ref.y(), 0.0);
            lift_angle = Vector3(0.0, 0.0, foot[swg].angle_ref.z());
        }
        else{
            // reference base orientation is set as the middle of feet orientation
            double ts = std::max(0.0, duration - ttl);
            
            base.angvel_ref.z() = yaw_angvel_des;
            base.angle_ref.z()  = angle_switch.z() + yaw_angvel_des*ts/*(ts/duration)*(turn/2)*/;
            base.ori_ref = FromRollPitchYaw(base.angle_ref);
            
            // configure support foot
            foot[sup].contact_ref = true;

            // set swing foot position
            if(!stepping || ttl > (1.0 - dsp_rate)*duration){
                foot[swg].contact_ref = true;
            }
            else{
                double ts   = std::max(0.0, (1.0 - dsp_rate)*duration - ttl);            //< time elapsed in ssp
                double tauv = (1.0 - dsp_rate)*duration;    //< duration of vertical movement
                double tauh = tauv - descend_rate*duration; //< duration of horizontal movement

                // cycloid swing profile
                double sv     = ts/tauv;
                double sh     = ts/tauh;
                double thetav = 2.0*pi*sv;
                double thetah = 2.0*pi*sh;
                double ch     = (sh < 1.0 ? (thetah - sin(thetah))/(2.0*pi) : 1.0);
                double cv     = (1.0 - cos(thetav))/2.0;
                double cv2    = (1.0 - cos(thetav/2.0))/2.0;
                double cw     = sin(thetah);

                // foot turning
                Vector3 foot_turn = land_angle - lift_angle;
                while(foot_turn.z() >  pi) foot_turn.z() -= 2.0*pi;
                while(foot_turn.z() < -pi) foot_turn.z() += 2.0*pi;

                // foot tilting
                Vector3 tilt = foot[swg].ori_ref*Vector3(0.0, swing_tilt, 0.0);

                foot[swg].pos_ref      = (1.0 - ch)*lift_pos + ch*land_pos;
                foot[swg].pos_ref.z() += (cv*(swing_height + 0.5*descend_depth) - cv2*descend_depth);
                foot[swg].angle_ref    = lift_angle + ch*foot_turn + cw*tilt;
                foot[swg].ori_ref      = FromRollPitchYaw(foot[swg].angle_ref);
                foot[swg].contact_ref  = false;
            }
        }
    }
    else{
        foot[0].contact_ref = true;
        foot[1].contact_ref = true;
    }

	// calc zmp for regulating dcm
    if(stepping){
	    centroid.zmp_target = foot[sup].pos_ref;
        centroid.dcm_target = (centroid.zmp_target + offset) + exp((timer.time - time_switch)/T)*(foot[sup].ori_ref*Vector3(dcm_offset[sup].x(), dcm_offset[sup].y(), 0.0));
    }
    else{
        centroid.zmp_target = (1.0/2.0)*(foot[0].pos_ref + foot[1].pos_ref);
        centroid.dcm_target = centroid.zmp_target + offset;
    }

    centroid.zmp_ref = centroid.zmp_target + dcm_ctrl_gain*(centroid.dcm_ref - centroid.dcm_target);

	// project zmp inside support region
	if(stepping){
		Vector3 zmp_local = foot[sup].ori_ref.conjugate()*(centroid.zmp_ref - foot[sup].pos_ref);
		for(int j = 0; j < 3; j++){
			zmp_local[j] = std::min(std::max(param.zmp_min[j], zmp_local[j]), param.zmp_max[j]);
		}
		centroid.zmp_ref = foot[sup].pos_ref + foot[sup].ori_ref*zmp_local;
	}

	// calc DCM derivative
	Vector3 dcm_d = (1/T)*(centroid.dcm_ref - (centroid.zmp_ref + offset)) + T*delta;

	// calc CoM acceleration
	centroid.com_acc_ref = (1/T)*(dcm_d - centroid.com_vel_ref);

	// update DCM
	centroid.dcm_ref += dcm_d*timer.dt;

	// calc CoM velocity from dcm
	centroid.com_vel_ref = (1/T)*(centroid.dcm_ref - centroid.com_pos_ref);

	// update CoM position
	centroid.com_pos_ref += centroid.com_vel_ref*timer.dt;

    // desired wrench to be applied to centroid
    centroid.force_ref  = param.total_mass*(centroid.com_acc_ref + Vector3(0.0, 0.0, param.gravity));
	centroid.moment_ref = Vector3(0.0, 0.0, tau_z);

    CalcForceDistribution(param, centroid, foot);
    
    if(enable_logging){
        if(!fileLogFk){
            fileLogFk   = fopen("log_fk.csv"  , "w");
            fileLogIk   = fopen("log_ik.csv"  , "w");
            fileLogCtrl = fopen("log_ctrl.csv", "w");
            fileLogStepFk[0] = fopen("log_step_fk0.csv", "w");
            fileLogStepFk[1] = fopen("log_step_fk1.csv", "w");
            fileLogStepIk[0] = fopen("log_step_ik0.csv", "w");
            fileLogStepIk[1] = fopen("log_step_ik1.csv", "w");
        
            fprintf(fileLogFk, "time, com_x, com_y, com_z, dcm_x, dcm_y, dcm_z, zmp_x, zmp_y, zmp_z, angle_x, angle_y, angle_z, rate_x, rate_y, rate_z, mom0_x, mom0_y, mom0_z, mom1_x, mom1_y, mom1_z\n");
            fprintf(fileLogIk, "time, com_x, com_y, com_z, dcm_x, dcm_y, dcm_z, zmp_x, zmp_y, zmp_z, angle_x, angle_y, angle_z, rate_x, rate_y, rate_z, mom0_x, mom0_y, mom0_z, mom1_x, mom1_y, mom1_z\n");
            fprintf(fileLogCtrl, "time, theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, Ld_x, Ld_y, Ld_z, delta_x, delta_y, delta_z, r_x, r_y, r_z\n");

            SaveFootstep(param, foot, 0, true );
            SaveFootstep(param, foot, 0, false);
            SaveFootstep(param, foot, 1, true );
            SaveFootstep(param, foot, 1, false);
        }

        fprintf(
            fileLogFk,
            "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            timer.time,
            centroid.com_pos.x(), centroid.com_pos.y(), centroid.com_pos.z(),
            centroid.dcm.x(), centroid.dcm.y(), centroid.dcm.z(),
            centroid.zmp.x(), centroid.zmp.y(), centroid.zmp.z(),
            base.angle.x(), base.angle.y(), base.angle.z(),
            base.angvel.x(), base.angvel.y(), base.angvel.z(),
            foot[0].moment.x(), foot[0].moment.y(), foot[0].moment.z(),
            foot[1].moment.x(), foot[1].moment.y(), foot[1].moment.z()
        );
        fprintf(
            fileLogIk,
            "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            timer.time,
            centroid.com_pos_ref.x(), centroid.com_pos_ref.y(), centroid.com_pos_ref.z(),
            centroid.dcm_ref.x(), centroid.dcm_ref.y(), centroid.dcm_ref.z(),
            centroid.zmp_ref.x(), centroid.zmp_ref.y(), centroid.zmp_ref.z(),
            base.angle_ref.x(), base.angle_ref.y(), base.angle_ref.z(),
            base.angvel_ref.x(), base.angvel_ref.y(), base.angvel_ref.z(),
            foot[0].moment_ref.x(), foot[0].moment_ref.y(), foot[0].moment_ref.z(),
            foot[1].moment_ref.x(), foot[1].moment_ref.y(), foot[1].moment_ref.z()
        );
        fprintf(
            fileLogCtrl,
            "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            timer.time, theta.x(), theta.y(), theta.z(), omega.x(), omega.y(), omega.z(), Ld.x(), Ld.y(), Ld.z(), delta.x(), delta.y(), delta.z(), r.x(), r.y(), r.z()
        );
    }
}

void ReactiveWalkingController::SaveFootstep(const Param& param, const vector<Foot>& foot, int idx, bool fk_or_ik){
    Vector2 vtx[4];
    vtx[0] = Vector2(param.zmp_min.x(), param.zmp_min.y());
    vtx[1] = Vector2(param.zmp_max.x(), param.zmp_min.y());
    vtx[2] = Vector2(param.zmp_max.x(), param.zmp_max.y());
    vtx[3] = Vector2(param.zmp_min.x(), param.zmp_max.y());

    Vector2 pos;
    double  angle;
    if(fk_or_ik){
        pos   = Vector2(foot[idx].pos.x(), foot[idx].pos.y());
        angle = foot[idx].angle.z();
    }
    else{
        pos   = Vector2(foot[idx].pos_ref.x(), foot[idx].pos_ref.y());
        angle = foot[idx].angle_ref.z();
    }
    
    FILE* file = (fk_or_ik ? fileLogStepFk[idx] : fileLogStepIk[idx]);
    for(int j = 0; j <= 4; j++){
        Vector2 v = pos + Eigen::Rotation2D<double>(angle)*vtx[j%4];
        fprintf(file, "%f, %f\n", v.x(), v.y());
    }
    fprintf(file, "\n");
}

}
}
