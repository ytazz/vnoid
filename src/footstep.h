#pragma once

#include <cnoid/Body>
#include <cnoid/SimpleController>

#include <vector>
#include <deque>
using namespace std;

namespace cnoid{
namespace vnoid{

class Footstep{
public:
	struct Step{
		struct Spec{
			double   stride;
            double   sway;
			double   spacing;
			double   turn;
			double   climb;
			double   duration;
            Vector3  cop_min;
            Vector3  cop_max;

			Spec();
		};

		Spec     spec;

		int      side;
		
		Vector3  footPos   [2];
		double   footOri   [2];
		Vector3  footVel   [2];
		double   footAngvel[2];

		Step();
	};

	vector<Step::Spec>  specs;
	deque <Step>        steps;

    int markerIndexBegin;  //< range of link indices for footstep marker
    int markerIndexEnd;

    Body*    ioBody;

public:
    void  GenerateStep (const Step::Spec& spec);
    void  GenerateSteps();
	void  Init         (SimpleControllerIO* io);
    void  UpdateMarkers();

	Footstep();
};

}
}