#pragma once

#include "../src/robot.h"

// 運動学 ステップ2
// fksolverとiksolverのヘッダファイルをインクルード

// 重心動力学 ステップ1
// stabilizerのヘッダファイルをインクルード

// 重心動力学 ステップ2
// visualizerのヘッダファイルをインクルード

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

    // 運動学 ステップ2
    // fksolverとiksolverをメンバ変数に追加

    // 重心動力学 ステップ1
    // stabilizerをメンバ変数に追加

    // 重心動力学 ステップ2
    // visualizerをメンバ変数に追加
    // Visualize()をメンバ関数に追加

    Joystick       joystick;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    MyRobot();

};

}
}