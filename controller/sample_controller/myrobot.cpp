#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace BipedCnoid{

const real_t inf = numeric_limits<real_t>::max();

MyRobot::MyRobot(int idx){
    index = idx;

    obstacles = new Obstacles();
        
    if(index == 0){
        path     = new Path();
        footstep = new Footstep();

	    pathPlanner = new PathPlanner();
        pathPlanner->obstacles = obstacles;
	    pathPlanner->path      = path;

        plannerCapt = new PlannerCapt();
        plannerCapt->footstep = footstep;
        plannerCapt->robot    = this;

        iksolver = new IkSolverSb ();

        planPeriod      = 1.0;
        minStepDuration = 0.5;

        lastPlanTime = -inf;
    }
    else{
        plannerSimple = new PlannerSimple();
        plannerSimple->robot = this;
    }
}

void MyRobot::Read(XMLNode* node){
    if(index == 0){
        iksolver   ->Read(node->GetNode("iksolver"   ));
        pathPlanner->Read(node->GetNode("pathplanner"));
        footstep   ->Read(node->GetNode("footstep"   ));
        plannerCapt->Read(node->GetNode("planner"    ));
        obstacles  ->Read(node->GetNode("obstacles"  ));

        node->Get(planPeriod     , ".plan_period"      );
        node->Get(minStepDuration, ".min_step_duration");
        node->Get(spacing        , ".spacing"          );
    }
    else{
        plannerSimple->Read(node->GetNode("planner"));
    }

	Robot::Read(node);
}

void MyRobot::Init(SimpleControllerIO* io){
	Robot::Init(io);

    if(index == 0){
        iksolver   ->Init();
        footstep   ->Init(io);
        pathPlanner->Init();
	    plannerCapt->Init(io);
    }
    else{
        plannerSimple->Init(io);
    }

}

void MyRobot::Control(){
    if(index == 0){
        if(time >= plannerCapt->standby_period && time >= lastPlanTime + planPeriod){
            pathPlanner->Plan();

            footstep->FromPath(path, pathPlanner->maxVelocity, minStepDuration, spacing);
            //for(Footstep::Step& step : footstep->steps){
            //    DSTR << step.footPos[0].x() << " " << step.footPos[0].y() << "  "
            //         << step.footPos[1].x() << " " << step.footPos[1].y() << endl;
            //}

            plannerCapt->Plan();

            lastPlanTime = time;
        }

        if(baseActuation){
            real_t t = std::max(0.0, time - plannerCapt->standby_period);
            
            base.pos_ref = pathPlanner->path->GetPos(t);
            base.pos_ref.z() = 1.0;
        }
        else{
            plannerCapt->FromRobot();
            plannerCapt->ToRobot();
        }
    }
    else{
        plannerSimple->ToRobot();
    }
    
	Robot::Control();
	
	Countup();

    //DSTR << "id: " << index << " time: " << time << endl;
}


}
}
