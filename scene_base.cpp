#include "scene_base.h"



//display, virtual function
void SceneBase::showScene() {
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	car0->showCar(BLACK);

	if (SHOWCIRCLE && car0->p_center)//
	{
		car0->showCircle();
	}
	EndBatchDraw();
	delay(DELAYTIME);
}

//Move straight:, moving in certain distance,// total_s positive, is total distance it travel
void SceneBase::uniformStraight(const double& total_s) {
	car0->updateStraightInfo();//update the straight motion info

	double s = 0.0; //how many distance do we need to travel
	while (s < total_s)
	{
		s += fabs(car0->speed);
		car0->moveStraightStep();
		obsMoveStep();
		showScene();//display
	}
	car0->coutInfo();
}

//Move straight:, acceralation, acceralate to certain speed, uniform acceleration (deceleration) linear motion.
void SceneBase::uniformAccBySpeed(const double& target_speed_y) {
	while (car0->pmidr->y > 0.0)
	{
		car0->moveStraightStep();
		obsMoveStep();
		if (fabs(car0->speed_y - target_speed_y) > fabs(car0->a_y))//if current speed and expect speed are not equal
		{
			car0->speed_y += car0->a_y; //accelerating
		}
		else //if current speed is close enough to expect speed
		{
			car0->speed_y = target_speed_y; //we directly let the speed equal expect speed
			car0->a_y = 0.0; //update the accelerating to 0

			if (target_speed_y == 0.0)//if parking, break the loop,otherwise p0 can move to 0.0
			{
				break;
			}
		}
		showScene();//display
	}
	car0->coutInfo();
}

/*Move straight : When traveling a specified distance,
the speed reaches the specified speed,uniform acceleration (deceleration) linear motion.)*/
void SceneBase::uniformAccByDis(const double& dis, const double& target_speed_y) {

	//calculate the acceleration
	car0->a_y = (pow(car0->speed_y, 2) - pow(target_speed_y, 2)) / dis / 2.0; //acceleration
	cout << "a_y = " << car0->a_y << ", dis = " << dis << endl;//print

	uniformAccBySpeed(target_speed_y);
}

//move straight:in specified time, reach to the specified speed,uniform acceleration (deceleration) linear motion.)
void SceneBase::uniformAccByTime(const double& target_speed_y, const double& target_time) {

	//calculate the acceleration
	double freq = target_time * 1000 / DELAYTIME;//how many loop do we need, count in frame, using the frame instead of real time, second.
	car0->a_y = (target_speed_y - car0->speed_y) / freq;//acceleration //speed change during each loop
	cout << "a_y = " << car0->a_y << endl; //print

	uniformAccBySpeed(target_speed_y);
}

/////////////turn/////////////////////

//turn, use radius and angle,
void SceneBase::carTurn(const int& turn_state, const double& R, const double& total_theta) {

	car0->updateTurnInfo(turn_state, R);//update the turning

	double theta = 0.0;
	while (theta < total_theta)
	{
		theta += fabs(car0->delta_theta);
		correctAngleError(car0->delta_theta, theta - total_theta);//error fix
		car0->carTurnStep();
		obsMoveStep();
		showScene();
	}
	car0->coutInfo();
}


//change lane
void SceneBase::laneChange(const Point& target_point, const int& type, const double& s) {

	double dis = car0->pmidr->distanceTo(target_point);
	Vec2d vec0(dis, car0->heading_theta + PI / 2.0);
	Vec2d vec(*car0->pmidr, target_point);
	double L = vec0.crossProd(vec) / dis / 2.0;
	double H = vec0.innerProd(vec) / dis / 2.0;

	if (fabs(L) < 1e-10)//target_point in front the car, not need to change lane
	{
		uniformStraight(car0->pmidr->distanceTo(target_point));
		return;
	}

	double R = (pow(L, 2) + pow(H, 2)) / fabs(L) / 2.0;//turning radius, L =0 is return
	double target_theta = asin(H / R);//target angle
	double target_delta_theta = fabs(car0->speed / R);//angular speed abs
	cout << "dis = " << dis << ", L = " << L << ", H = " << H << ", R = " << R << ", target_theta = " << target_theta / PI << ", target_delta_theta = " << target_delta_theta / PI << endl;

	if (L > 0.0)//left lane
	{
		car0->delta_theta = target_delta_theta;
		carTurn(TurnDirection::TurnLeft, R, target_theta);

		if (type == singleType)
		{
			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
		else
		{
			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);

			uniformStraight(s);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);
		}
	}
	else if (L < 0.0)//right lane
	{
		car0->delta_theta = -target_delta_theta;
		carTurn(TurnDirection::TurnRight, R, target_theta);

		if (type == singleType)
		{
			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);
		}
		else
		{
			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			uniformStraight(s);//if is overtaking, we need straight move for certain distance

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
	}
}