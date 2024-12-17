#pragma once

#include "../src/robot_demo.h"
#include "fksolver.h"
#include "iksolver.h"
#include "bvh.h"
#include "stabilizer.h"
#include "visualizer.h"

/*
 * Dance demo 1
 * 
 * Robot plays motion data in BVH format
 * 
 * Upper body tracks motion data by simple retargeting.
 * Lower body is controlled by the stabilizer (no motion tracking).
 * Current implementation of retargeting is more or less specialized to CMU Mocap Dataset.
 * The data do not come with vnoid, so please download and copy them to the motion directory by your self.
 * 
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public RobotDemo{
public:
    Timer          timer;
    Param          param;
    Base           base;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;
    
    FkSolver    fk_solver;
    IkSolver    ik_solver;
	Bvh         bvh;
    Stabilizer  stabilizer;
    Visualizer  viz;
	
    string baseName ;
    string chestName;
    string headName ;
    string handName[2];
    string shoulderName[2];
    string elbowName[2];
    string middleName[2];
    
    Bvh::Node* baseNode ;
    Bvh::Node* chestNode;
    Bvh::Node* headNode ;
    Bvh::Node* handNode[2];
    Bvh::Node* shoulderNode[2];
    Bvh::Node* elbowNode[2];
    Bvh::Node* middleNode[2];

    double  handScale;
    Vector3 handOffset;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    void Visualize();
	
    MyRobot();

};

}
}