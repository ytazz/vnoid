#include "footstep.h"

namespace cnoid{
namespace vnoid{

Step::Step(){
    stride   = 0.0;
    sway     = 0.0;
	spacing  = 0.0;
	turn     = 0.0;
	climb    = 0.0;
	duration = 0.0;
    
    for(int i = 0; i < 2; i++){
		foot_pos   [i] = Vector3(0.0, 0.0, 0.0);
		foot_ori   [i] = 0.0;
		foot_vel   [i] = Vector3(0.0, 0.0, 0.0);
		foot_angvel[i] = 0.0;
	}
	zmp = Vector3(0.0, 0.0, 0.0);
	dcm = Vector3(0.0, 0.0, 0.0);
}

}
}