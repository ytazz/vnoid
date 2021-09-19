#include "footstep.h"

namespace cnoid{
namespace vnoid{

const double eps = 1.0e-10;

///////////////////////////////////////////////////////////////////////////////////////////////////

Footstep::Step::Spec::Spec(){
	stride   = 0.0;
    sway     = 0.0;
	spacing  = 0.0;
	turn     = 0.0;
	climb    = 0.0;
	duration = 0.0;
    cop_min  = Vector3(-0.1, -0.05, -10.0);
    cop_max  = Vector3( 0.1,  0.05,  10.0);
}

Footstep::Step::Step(){
	for(int i = 0; i < 2; i++){
		footPos   [i] = Vector3(0.0, 0.0, 0.0);
		footOri   [i] = 0.0;
		footVel   [i] = Vector3(0.0, 0.0, 0.0);
		footAngvel[i] = 0.0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Footstep::Footstep(){
    markerIndexBegin = -1;
    markerIndexEnd   = -1;
}

void Footstep::GenerateStep(const Step::Spec& spec){
	Step& st0 = steps.back();
	Step  st1;

	st0.spec = spec;

	int sup =  st0.side;
	int swg = !st0.side;

	double  dtheta = spec.turn;
	double  l      = spec.stride;
    double  d      = spec.sway;
	double  w      = (sup == 0 ? 1.0 : -1.0) * spec.spacing;
	double  dz     = spec.climb;
	Vector3 dprel;

	if(std::abs(dtheta) < eps){
		dprel = Vector3(l, w + d, dz);
	}
	else{
		double r = l/dtheta;
		dprel = Vector3(
			(r - w/2.0 - d)*sin(dtheta),
			(r + w/2.0) - (r - w/2.0 - d)*cos(dtheta),
			dz);
	}

	st1.side = !st0.side;
	st1.footPos[sup] = st0.footPos[sup];
	st1.footOri[sup] = st0.footOri[sup];
	st1.footPos[swg] = st0.footPos[sup] + Eigen::AngleAxisd(st0.footOri[sup], Vector3::UnitZ())*dprel;
	st1.footOri[swg] = st0.footOri[sup] + dtheta;

	steps.push_back(st1);
	
}

void Footstep::GenerateSteps(){
    steps.clear();

    steps.push_back(Step());
	steps[0].side = 0;
	steps[0].footPos[0] = Vector3(0.0, -specs[0].spacing/2.0, 0.0);
	steps[0].footPos[1] = Vector3(0.0,  specs[0].spacing/2.0, 0.0);
	steps[0].footOri[0] = 0.0;
	steps[0].footOri[1] = 0.0;

	for(Step::Spec& spec : specs){
		GenerateStep(spec);
	}
}

void Footstep::Init(SimpleControllerIO* io){
    // configure footstep marker links
    ioBody = io->body();
	
    int fs0 = markerIndexBegin;
    int fs1 = markerIndexEnd  ;
    if(0 <= fs0 && fs0 < fs1 && fs1 <= ioBody->numLinks()){
        for(int i = fs0; i < fs1; i++){
            ioBody->link(i)->setActuationMode(cnoid::Link::LinkPosition);
            io->enableIO(ioBody->link(i));
        }
    }

    GenerateSteps();
}

void Footstep::UpdateMarkers(){
    // update marker position
    int fs0 = markerIndexBegin;
    int fs1 = markerIndexEnd  ;
    if(0 <= fs0 && fs0 < fs1 && fs1 <= ioBody->numLinks()){
        for(int i = 0; i < std::min(fs1 - fs0, (int)steps.size()); i++){
            Link* lnk = ioBody->link(i + fs0);
            Footstep::Step& step = steps[i];
            lnk->p() = step.footPos[step.side];
            lnk->R() = AngleAxis(step.footOri[step.side], Vector3::UnitZ()).matrix();
        }
    }
}

}
}