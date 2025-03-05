#pragma once
#include "scene_base.h"

//static obstancle
class StaticObs : public SceneBase {

public:
	StaticObs();
	void showScene();//display
	bool planning_process() override;//process

public:
	unique_ptr<Cone> cone;//cone
	double start_dis = 200.0;//distance start to obstacle avoidance
};

//overtaking
class OvertakeObs : public SceneBase {

public:
	OvertakeObs();
	void obsMoveStep();
	void showScene();//display
	bool planning_process() override;//process

public:
	unique_ptr<CarNormal> carObs;//obstacle car
	double start_dis = 0.0;//distance start to obstacle avoidance
};

//oncoming traffic
class MeetingObs : public SceneBase {

public:
	MeetingObs();
	void obsMoveStep();
	void showScene();//display
	bool planning_process() override;//process

public:
	unique_ptr<CarNormal> carObs;//obstacle car
	double start_dis = 200.0;//distance start to obstacle avoidance
};