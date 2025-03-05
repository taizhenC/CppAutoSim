#pragma once
#include "road.h"
#include "car.h"

//change lane
enum LaneChangeType {

	singleType,//single lane
	doubleType//double lane
};



class SceneBase {

public:
	virtual ~SceneBase() = default;// virtual destructor
	virtual void showScene();//display, virtual function, derived is not provided, use base instead
	virtual void obsMoveStep() {}//virtual function, obstacle single frame motion, describe the simple motion of obstacle.
	virtual bool planning_process() = 0;//full process, pure virtual function

	//straight motion
	void uniformStraight(const double& total_s); //Move straight:, moving in certain distance,// total_s positive, is total distance it travel
	void uniformAccBySpeed(const double& target_speed_y); // Move straight:, acceralation, acceralate to certain speed, linear acceralating motion
	void uniformAccByDis(const double& dis, const double& target_speed_y); /*(Move straight: When traveling a specified distance, the speed reaches the specified speed,																	uniform acceleration (deceleration) linear motion.)*/
	void uniformAccByTime(const double& target_speed, const double& target_time); //move straight:in specified time, reach to the specified speed,uniform acceleration (deceleration) linear motion.)
	
	//turing motion
	void carTurn(const int& turn_state, const double& R, const double& total_theta);//turn, use radius and angle, 
	void laneChange(const Point& target_point, const int& type, const double& s = 0.0);//change lane, single displacement line,double displacement line

public:
	unique_ptr<RoadBase> road0;//road base pointer
	unique_ptr<CarBase> car0;//case base pointer
	double speedlimit = -6.0;//limiting speed, can be + -
};
