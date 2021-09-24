#include "stabilizer.h"

#include "robot.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

Stabilizer::Stabilizer(){
    min_contact_force        = 1.0;
	force_ctrl_damping       = 0.0;
	force_ctrl_gain          = 0.0;
	force_ctrl_limit         = 0.0;
	moment_ctrl_damping      = 0.0;
	moment_ctrl_gain         = 0.0;
	moment_ctrl_limit        = 0.0;
	orientation_ctrl_gain_p  = 0.0;
	orientation_ctrl_gain_d  = 0.0;
	swing_height_adjust_rate = 0.0;
}

void Stabilizer::CalcZmp(const Param* param, Centroid* centroid, Foot* foot){
    // get actual force from the sensor
	for(int i = 0; i < 2; i++){
		// set contact state
		foot[i].contact = (foot[i].force[2] >= min_contact_force);

		// measure continuous contact duration
		if(foot[i].contact){
			foot[i].contact_duration += param->dt;
			foot[i].zmp = Vector3(-foot[i].moment[1]/foot[i].force[2], foot[i].moment[0]/foot[i].force[2], 0.0);
		}
		else{
			foot[i].contact_duration = 0.0;
			foot[i].zmp = Vector3(0.0, 0.0, 0.0);
		}
	}

	// both feet not in contact
	if(!foot[0].contact && !foot[1].contact){
		foot[0].balance = 0.5;
		foot[1].balance = 0.5;
		centroid->zmp = Vector3(0.0, 0.0, 0.0);
	}
	else{
		double f0 = std::max(0.0, foot[0].force[2]);
		double f1 = std::max(0.0, foot[1].force[2]);
		foot[0].balance = f0/(f0 + f1);
		foot[1].balance = f1/(f0 + f1);
		centroid->zmp =
			     (foot[0].balance) * (foot[0].pos_ref + foot[0].ori_ref * foot[0].zmp)
	           + (foot[1].balance) * (foot[1].pos_ref + foot[1].ori_ref * foot[1].zmp);
	}
}

void Stabilizer::CalcForceDistribution(const Param* param, Centroid* centroid, Foot* foot){
	// switch based on contact state
	if(!foot[0].contact_ref && !foot[1].contact_ref){
		foot[0].balance_ref = 0.5;
		foot[1].balance_ref = 0.5;
		foot[0].zmp_ref = Vector3(0.0, 0.0, 0.0);
		foot[1].zmp_ref = Vector3(0.0, 0.0, 0.0);
	}
	if( foot[0].contact_ref && !foot[1].contact_ref){
		foot[0].balance_ref = 1.0;
		foot[1].balance_ref = 0.0;
		foot[0].zmp_ref = foot[0].ori_ref.conjugate() * ((centroid->zmp_ref + centroid->zmp_mod) - foot[0].pos_ref);
		foot[1].zmp_ref = Vector3(0.0, 0.0, 0.0);
	}
	if(!foot[0].contact_ref &&  foot[1].contact_ref){
		foot[0].balance_ref = 0.0;
		foot[1].balance_ref = 1.0;
		foot[0].zmp_ref = Vector3(0.0, 0.0, 0.0);
		foot[1].zmp_ref = foot[1].ori_ref.conjugate() * ((centroid->zmp_ref + centroid->zmp_mod) - foot[1].pos_ref);
	}
	if( foot[0].contact_ref &&  foot[1].contact_ref){
		//
		Vector2 b;
		Vector3 pdiff  = foot[1].pos_ref - foot[0].pos_ref;
		double  pdiff2 = pdiff.squaredNorm();
		const double eps = 1.0e-10;
		if(pdiff2 < eps){
			b[0] = b[1] = 0.5;
		}
		else{
			b[0] = (pdiff.dot(foot[1].pos_ref - (centroid->zmp_ref + centroid->zmp_mod)))/pdiff2;
			b[0] = std::min(std::max(0.0, b[0]), 1.0);
			b[1] = 1.0 - b[0];
		}

		foot[0].balance_ref = b[0];
		foot[1].balance_ref = b[1];

		Vector3 zmp_proj = b[0]*foot[0].pos_ref + b[1]*foot[1].pos_ref;

		double b2 = b.squaredNorm();
		foot[0].zmp_ref = (b[0]/b2) * (foot[0].ori_ref.conjugate() * ((centroid->zmp_ref + centroid->zmp_mod) - zmp_proj));
		foot[1].zmp_ref = (b[1]/b2) * (foot[1].ori_ref.conjugate() * ((centroid->zmp_ref + centroid->zmp_mod) - zmp_proj));
	}

	for(int i = 0; i < 2; i++){
		// force and moment to realize desired Zmp
		foot[i].force_ref     =  foot[i].ori_ref.conjugate() * (foot[i].balance_ref * centroid->force_ref);
		foot[i].moment_ref[0] =  foot[i].force_ref[2] * foot[i].zmp_ref[1];
		foot[i].moment_ref[1] = -foot[i].force_ref[2] * foot[i].zmp_ref[0];
		foot[i].moment_ref[2] =  foot[i].balance_ref * centroid->moment_ref[2];

		//DSTR << foot[i].force_ref[2] << " " << foot[i].zmp_ref[0] << " " << foot[i].zmp_ref[1] << endl;
	}
}

void Stabilizer::Step(const Param* param, Centroid* centroid, Base* base, Foot* foot){
	// copy reference values from iksolver
	centroid->force_ref[0] = param->total_mass* centroid->com_acc_ref[0];
	centroid->force_ref[1] = param->total_mass* centroid->com_acc_ref[1];
	centroid->force_ref[2] = param->total_mass*(centroid->com_acc_ref[2] + param->gravity);
	centroid->moment_ref   = Vector3(0.0, 0.0, 0.0);

	// feedback base orientation to zmp
	{
		Vector3 m(0.0, 0.0, 0.0);
		m[0] = orientation_ctrl_gain_p * (base->angle_ref[0] - base->angle[0]) + orientation_ctrl_gain_d * (base->angvel_ref[0] - base->angvel[0]);
		m[1] = orientation_ctrl_gain_p * (base->angle_ref[1] - base->angle[1]) + orientation_ctrl_gain_d * (base->angvel_ref[1] - base->angvel[1]);

		centroid->moment_mod = base->ori_ref * m;
		centroid->zmp_mod    = base->ori_ref * ((1.0/centroid->force_ref[2])*Vector3(-m[1], m[0], 0.0));
	}

	// calculate desired forces from desired zmp
	CalcForceDistribution(param, centroid, foot);

	for(int i = 0; i < 2; i++){
		foot[i].pos_mod    = Vector3(0.0, 0.0, 0.0);
		foot[i].angle_mod  = Vector3(0.0, 0.0, 0.0);
		foot[i].vel_mod    = Vector3(0.0, 0.0, 0.0);
		foot[i].angvel_mod = Vector3(0.0, 0.0, 0.0);
	}

	for(int i = 0; i < 2; i++){
		// ground reaction force control
		if( foot[i].contact ){
			foot[i].force_error[2] = foot[i].force_ref[2] - foot[i].force[2];
			foot[i].dpos[2] += (-force_ctrl_damping*foot[i].dpos[2] + force_ctrl_gain*foot[i].force_error[2])*param->dt;
			foot[i].dpos[2] = std::min(std::max(-force_ctrl_limit, foot[i].dpos[2]), force_ctrl_limit);

			foot[i].moment_error[0] = foot[i].moment_ref[0] - foot[i].moment[0];
			foot[i].moment_error[1] = foot[i].moment_ref[1] - foot[i].moment[1];
			foot[i].drot[0] += (-moment_ctrl_damping*foot[i].drot[0] + moment_ctrl_gain*foot[i].moment_error[0])*param->dt;
			foot[i].drot[1] += (-moment_ctrl_damping*foot[i].drot[1] + moment_ctrl_gain*foot[i].moment_error[1])*param->dt;
			foot[i].drot[0] = std::min(std::max(-moment_ctrl_limit, foot[i].drot[0]), moment_ctrl_limit);
			foot[i].drot[1] = std::min(std::max(-moment_ctrl_limit, foot[i].drot[1]), moment_ctrl_limit);

			// feedback to desired foot pose
			foot[i].pos_mod   += -foot[i].dpos;
			foot[i].angle_mod += -foot[i].drot;
			//foot[i].vel_mod    += -foot[i].dpos;
			//foot[i].angvel_mod += -foot[i].drot;
		}
	}

	// swing foot height compensation
	if( ( foot[0].contact_ref && !foot[1].contact_ref) ||
	    (!foot[0].contact_ref &&  foot[1].contact_ref) ){
		int sup = (foot[0].contact_ref ? 0 : 1);
		int swg = !sup;
		Vector3 diff = base->ori_ref.conjugate()*(foot[swg].pos_ref - foot[sup].pos_ref);
		foot[swg].pos_mod[2] += swing_height_adjust_rate*(diff[0]*base->angle[1] - diff[1]*base->angle[0]);
	}
}

}
}
