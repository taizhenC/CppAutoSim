#include "scene_obs_pass.h"

////////////static obstancle /////////////////
StaticObs::StaticObs() {

	road0 = make_unique<RoadNormal>(250.0);
	cone = make_unique<Cone>(SHEIGHT / 2.0, SWIDTH / 2.0, 50.0);

	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0);
	car0->speed = -4.0;

	car0->coutInfo();
	showScene();
	system("pause");
}

//display
void StaticObs::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	cone->showCone();
	car0->showCar(BLACK);

	if (SHOWCIRCLE && car0->p_center)//draw the trajectory
	{
		car0->showCircle();
	}

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool StaticObs::planning_process()
{
	double start_line = cone->p_center->y + cone->r + start_dis + car0->car_length;//position that start to obstacle avoidance
	uniformStraight(car0->pmidr->y - start_line);

	//calculate 
	double dis_l_L = cone->p_center->x - cone->r - road0->left_boundary;//left instance
	double dis_r_L = road0->right_boundary - cone->p_center->x - cone->r;//right instance
	double target_l_x = road0->left_boundary + dis_l_L / 2.0;//left target x point 
	double target_r_x = road0->right_boundary - dis_r_L / 2.0;//right target x point
	double target_y = cone->p_center->y;//left instance or right instance y center
	Point target_l_point(target_l_x, target_y);//left target point
	Point target_r_point(target_r_x, target_y);//right target point
	cout << "dis_l_L = " << dis_l_L << ", dis_r_L = " << dis_r_L << ", target_l_x= " << target_l_x << ", target_r_x= " << target_r_x << ", target_y= " << target_y << endl;

	if (dis_l_L > car0->car_width * 1.2)//if left space is enough, change from left
	{
		laneChange(target_l_point, LaneChangeType::doubleType);
	}
	else//if left space not enough
	{
		if (dis_r_L > car0->car_width * 1.2)//if right is enough space, change from right
		{
			laneChange(target_r_point, LaneChangeType::doubleType);
		}
		else//if right also not enough space
		{
			cout << "there is no enough space, stop at obstancle" << endl;
			double stopline = cone->p_center->y + cone->r + 50.0;
			uniformAccByDis(car0->pmidf->y - stopline, 0.0);
			return false;
		}
	}

	car0->updatePmidr();
	uniformStraight(car0->pmidr->y - 0.0);
	return true;
}

//////////////////// overtakeing ///////////////////////

OvertakeObs::OvertakeObs() {

	double Rwidth = 100.0;
	road0 = make_unique<RoadDoubleLane>(Rwidth);
	carObs = make_unique<CarNormal>(SWIDTH / 2.0 + Rwidth / 2.0, SHEIGHT - 400.0, 0.0, 50.0, 100.0);
	car0 = make_unique<CarNormal>(SWIDTH / 2.0 + Rwidth / 2.0, SHEIGHT - 70.0, 0.0, 40.0, 80.0);

	carObs->speed = -2.0;
	car0->speed = -6.0;

	carObs->updateStraightInfo();//car0bs move for straight

	car0->coutInfo();
	carObs->coutInfo();

	showScene();
	system("pause");
}

void OvertakeObs::obsMoveStep() {

	carObs->moveStraightStep();
}

//display
void OvertakeObs::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(RED);
	car0->showCar(BLACK);

	if (SHOWCIRCLE && car0->p_center)//draw the trajectory
	{
		car0->showCircle();
	}

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool OvertakeObs::planning_process()
{
	double delta_speed = fabs(car0->speed) - fabs(carObs->speed);//speed different, 
	cout << "delta_speed = " << delta_speed << endl;
	if (delta_speed <= 0.0)//if main car slower, we cant overtake
	{
		cout << "Main car are too slow, can't overtake" << endl;
		return false;
	}

	double dis_l_L = carObs->plr->x - road0->left_boundary;//left instance
	double target_l_x = road0->left_boundary + dis_l_L / 2.0;//left target x point
	cout << "dis_l_L = " << dis_l_L << ", target_l_x = " << target_l_x << endl;
	if (dis_l_L <= car0->car_width * 1.2)
	{
		cout << "There is no enough space for overtake" << endl;
		return false;
	}

	start_dis = car0->car_length * 3.5;
	double dis0 = car0->pmidr->y - carObs->pmidr->y;//init distance with target car
	cout << "start_dis = " << start_dis << ", dis0 = " << dis0 << endl;
	if (dis0 < start_dis)
	{
		cout << "too close at the front car, no enough space for overtake" << endl;
		return false;
	}

	uniformStraight(dis0 - start_dis);//go straight until to point that can overtake
	double time_lane_change = dis0 / delta_speed;//the time spend to the parallels with target car
	double dis_target = carObs->car_length;//target distance, at least one car space
	double target_y = car0->pmidr->y + car0->speed * time_lane_change;//toward the parallels position, left target y point
	double time_straight = dis_target / delta_speed;//move straight time
	double s = fabs(car0->speed) * time_straight;//move straight distance
	cout << "dis_target = " << dis_target << ", target_y = " << target_y << ", s = " << s << endl;

	Point target_l_point(target_l_x, target_y);//left target point
	laneChange(target_l_point, LaneChangeType::doubleType, s);//move the to parallel position with the target car, move straight until have at least one car space, than change to right lane

	car0->updatePmidr();
	uniformStraight(car0->pmidr->y - 0.0);
	return true;
}

/////////////////// oncoming traffic /////////////////////

MeetingObs::MeetingObs() {

	road0 = make_unique<RoadNormal>(100.0);
	carObs = make_unique<CarNormal>(SWIDTH / 2.0, 50.0, PI, 50.0, 100.0);
	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0, 0.0, 40.0, 80.0);
	carObs->speed = -3.0;
	car0->speed = -4.0;

	carObs->updateStraightInfo();//let carobs ready to move straight

	car0->coutInfo();
	carObs->coutInfo();
	showScene();
	system("pause");
}

void MeetingObs::obsMoveStep() {

	carObs->moveStraightStep();
}

void MeetingObs::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(RED);
	car0->showCar(BLACK);

	if (SHOWCIRCLE && car0->p_center)//draw the trajectory
	{
		car0->showCircle();
	}

	EndBatchDraw();
	delay(DELAYTIME);
}

bool MeetingObs::planning_process()  {

	double total_speed = fabs(car0->speed) + fabs(carObs->speed);//speed sum,
	double dis0 = fabs(car0->pmidf->y - carObs->pmidf->y);//init distance
	if (total_speed <= 0.0)
	{
		cout << "both car are static" << endl;
		return false;
	}

	double dis_r_L = road0->right_boundary - carObs->plr->x;//right instance
	double target_r_x = road0->right_boundary - dis_r_L / 2.0;//right target x point
	cout << "dis_r_L = " << dis_r_L << ", target_r_x = " << target_r_x << endl;
	if (dis_r_L <= car0->car_width * 1.2)
	{
		cout << "there is no enough space at right" << endl;
		return false;
	}

	double time = dis0 / total_speed;//collision time
	double meeting_point_y = car0->pmidf->y + car0->speed * time;//collision y point
	double s0 = car0->pmidf->y - (meeting_point_y + start_dis);//distance between car and obstancle
	cout << "total_speed = " << total_speed << ", dis0 = " << dis0 << ", meeting_point_y = " << meeting_point_y << ", s0 = " << s0 << endl;
	if (s0 < 0.0)
	{
		cout << "Too close, can't dodge it" << endl;
		return false;
	}

	uniformStraight(s0);//move straight until to the we can change lanes
	Point target_r_point(target_r_x, meeting_point_y);//right target point
	laneChange(target_r_point, LaneChangeType::doubleType);

	car0->updatePmidr();
	uniformStraight(car0->pmidr->y - 0.0);
	return true;
}