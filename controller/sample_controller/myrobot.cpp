#include "myrobot.h"

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(int idx){
    index = idx;

}

void MyRobot::Init(SimpleControllerIO* io){
	Robot::Init(io);
}

void MyRobot::Control(){

}


}
}
