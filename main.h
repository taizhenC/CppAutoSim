#pragma once
#include "scene_straight.h"
#include "scene_obs_pass.h"

enum PlanType {
	StraightStopObsType,////Obstacle stop
	StraightStationType,//parking stop
	StraightFollowType, //follow vehicle
	StraightCrosswalkType, //cross walk road

	//obstacle avoidance
	ObsPassStaticType,//static obstancle
	ObsPassOvertakeType,//overtaking
	ObsPassMeetingType,//oncoming traffic
};