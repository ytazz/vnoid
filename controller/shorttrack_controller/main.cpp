#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/BasicSensors>
#include <cnoid/RangeSensor>

using namespace cnoid;

/*
 *  Time meaurement for shorttrack field
 * 
 *  - lidar0: installed at the start line
 *  - lidar1: installed 10cm behind the goal line
 *  - lidar2: installed at the goal line
 *
 *  time measuring sequence
 *  - lidar0 on  -> start timer
 *  - lidar1 on
 *  - lidar2 on
 *  - lidar1 off
 *  - lidar2 off -> stop timer
 */

const int segMap[10][7] = {
	{1, 1, 1, 0, 1, 1, 1},
	{0, 0, 0, 0, 0, 1, 1},
	{0, 1, 1, 1, 1, 1, 0},
	{0, 0, 1, 1, 1, 1, 1},
	{1, 0, 0, 1, 0, 1, 1},
	{1, 0, 1, 1, 1, 0, 1},
	{1, 1, 1, 1, 1, 0, 1},
	{0, 0, 1, 0, 0, 1, 1},
	{1, 1, 1, 1, 1, 1, 1},
	{1, 0, 1, 1, 1, 1, 1}
};

const int numSegs   = 7;
const int numDigits = 5;

class VnoidShorttrackController : public SimpleController{
public:
	RangeSensor*  rangeSensor[3];
	Link*         joints[numSegs*numDigits];

	bool   on[3];
	int    state;
	double time;

	SimpleControllerIO* io;

	enum{
		PreStart,
		Started,
		PassingLine1,
		PassingLine2,
		PassedLine1,
		PassedLine2,
	};
	
public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
		this->io = io;

		rangeSensor[0] = io->body()->findDevice<RangeSensor>("rangeSensor0");
		rangeSensor[1] = io->body()->findDevice<RangeSensor>("rangeSensor1");
		rangeSensor[2] = io->body()->findDevice<RangeSensor>("rangeSensor2");
		
		io->enableInput(rangeSensor[0]);
		io->enableInput(rangeSensor[1]);
		io->enableInput(rangeSensor[2]);

		for(int i = 0; i < numDigits*numSegs; i++){
			joints[i] = io->body()->joint(i);
			joints[i]->setActuationMode(cnoid::Link::JointVelocity);
			io->enableIO(joints[i]);
			io->enableInput(joints[i], cnoid::Link::JointVelocity);
		}

		state = PreStart;

		return true;
	}

	virtual bool control()	{
		const double rmax = 27.5;
		for(int i = 0; i < 3; i++){
			on[i] = false;
			for(int j = 0; j < rangeSensor[i]->rangeData().size(); j++){
				if(rangeSensor[i]->rangeData()[j] < rmax){
					on[i] = true;
					break;
				}
			}
		}

		switch(state){
		case PreStart:
			if(on[0]){
				state = Started;
				time = 0.0;
			}
			break;
		case Started:
			if(on[1]){
				state = PassingLine1;
			}
			break;
		case PassingLine1:
			if(on[2]){
				state = PassingLine2;
			}
			break;
		case PassingLine2:
			if(!on[1]){
				state = PassedLine1;
			}
			break;
		case PassedLine1:
			if(!on[2]){
				state = PassedLine2;
			}
			break;
		default:
			break;
		}

		// display time
		// digit [4]:[3][2]:[1][0]
		//       mininutes : seconds : 1/100 seconds
		int digit[numDigits];
		double t = time;
		digit[4] = (int)(t /  0.01)%10; t -= 0.01*digit[4];
		digit[3] = (int)(t /  0.1 )%10; t -= 0.1 *digit[3];
		digit[2] = (int)(t /  1.0 )%10; t -= 1.0 *digit[2];
		digit[1] = (int)(t / 10.0 )%10;
		digit[0] = (digit[1] / 6) % 10;
		digit[1] = digit[1] % 6;
		
		int idx = 0;
		const double K = 100.0;
		for(int i = 0; i < numDigits; i++){
			for(int j = 0; j < numSegs; j++){
				double qref = (segMap[digit[i]][j] ? 0.0 : -0.1);
				joints[idx]->dq_target() = K*(qref - joints[idx]->q());
				idx++;
			}
		}

		if(state != PreStart && state != PassedLine2){
			time += io->timeStep();
		}

		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidShorttrackController)
