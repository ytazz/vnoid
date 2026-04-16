#pragma once

#include "../src/robot.h"

/*
 * 運動学 ステップ2
 * fksolverとiksolverのヘッダファイルをインクルード
 */
#include "fksolver.h"
#include "iksolver.h"

/*
 * 重心動力学 ステップ1
 * stabilizerのヘッダファイルをインクルード
 */
#include "stabilizer.h"

/*
 * 重心動力学 ステップ2
 * visualizerのヘッダファイルをインクルード
 */
#include "visualizer.h"

/*
 * 歩行パターン生成 ステップ1
 * footstep_plannerとstepping_controllerのヘッダファイルをインクルード
 */
#include "footstep_planner.h"
#include "stepping_controller.h"


#include <cnoid/Joystick>

/*
 * Tutorial 1
 * 
 * Use this code template when you go through this tutorial.
 * see myrobot_complete.[h|cpp] for complete code.
 * 
 */

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
	Timer          timer;
    Param          param;
    Base           base;
    Centroid       centroid;
    vector<Hand>   hand;
    vector<Foot>   foot;
    vector<Joint>  joint;

    /*
     * 運動学 ステップ2
     * fksolverとiksolverをメンバ変数に追加
     */
    FkSolver fk_solver;
    IkSolver ik_solver;

    /*
     * 重心動力学 ステップ1
     * stabilizerをメンバ変数に追加
     */
    Stabilizer stabilizer;

    /*
     * 重心動力学 ステップ2
     * visualizerをメンバ変数に追加
     * Visualize()をメンバ関数に追加
     */
    Visualizer  visualizer;
    void Visualize();

    /*
     * 歩行パターン生成 ステップ1
     * footstep_plannerとstepping_controllerをメンバ変数に追加
     */
    Footstep            footstep;    
    Footstep            footstep_buffer;
    FootstepPlanner     footstep_planner;
    SteppingController  stepping_controller;

    Joystick       joystick;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    MyRobot();

};

}
}