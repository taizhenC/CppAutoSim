#pragma once
#include "scene_base.h"



//Obstacle stop
class StraightStopObs : public SceneBase
{
public:
	StraightStopObs();
	void showScene();//display
	bool planning_process() override;

public:
	unique_ptr<Cone> cone;//cone
	double safedis = 50.0;//stop distance,
};

//stop point
class StraightStation : public SceneBase {

public:
	StraightStation();
	void showScene();//display
	bool planning_process() override;

public:
	unique_ptr<Point> station;//station point
	int stop_time = 3;//stop time,s
};

class StraightFollow : public SceneBase {

public:
	StraightFollow();
	void showScene();//display
	bool planning_process() override;

public:
	unique_ptr<CarNormal> carObs;//obstacle car
	double safedis = 120.0;//maintain distance between vehicles
};

//Zebra crossing
class StraightCrosswalk : public SceneBase {

public:
	StraightCrosswalk();
	bool peopleInCross();//check if there have pedestrian
	void showScene();//display
	bool planning_process() override;

public:
	int people_num = 5;//people
	vector<unique_ptr<Person>> people;//pedestrian
	double speedlimit_cross = -3.0;//limit speed in pedestrian walk

};