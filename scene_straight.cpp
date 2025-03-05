#include "scene_straight.h"



//Obstacle stop
StraightStopObs::StraightStopObs() {

	road0 = make_unique<RoadNormal>();
	cone = make_unique<Cone>(SWIDTH / 2.0, SWIDTH / 4.0, 50.0);//place the cone

	//placing the car
	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0);
	car0->speed_y = -5.0;

	car0->coutInfo();
	showScene();
	system("pause");
}

void StraightStopObs::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	cone->showCone();
	car0->showCar(BLACK);

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool StraightStopObs::planning_process() {

	double stopline = cone->p_center->y + cone->r + safedis;
	double dis = car0->pmidf->y - stopline;
	uniformAccByDis(dis, 0.0);
	/*double stopline = cone->p_center->y + cone->r + safedis;
	uniformAccByDis(car0->pmidf->y - stopline, 0.0);*/
	return true;
}

//aaaaaaaaaaaaaaaaaaaaaaaa Station Scene aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
StraightStation::StraightStation() {

	road0 = make_unique<RoadNormal>(); //init the road
	station = make_unique<Point>(SWIDTH / 2.0, SHEIGHT / 2.0); //create a point to be station

	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0);
	car0->speed_y = -5.0;

	car0->coutInfo();
	showScene();
	system("pause");
}

//display
void StraightStation::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	station->showPoint();
	car0->showCar(BLACK);

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool StraightStation::planning_process() {
	uniformAccByDis(car0->pmid->y - station->y, 0.0);// decelerate speed
	delay(stop_time * 1000);// stop and wait
	uniformAccByTime(speedlimit, 2.0);//exit station,accelerate
	return true;
}

//aaaaaaaaaaaaaaaaaaaa Following Scene aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
StraightFollow::StraightFollow() {

	road0 = make_unique<RoadNormal>();
	carObs = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT / 2.0, 0.0, 50.0, 100.0);
	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0);
	carObs->speed_y = -2.0;
	car0->speed_y = -5.0;

	car0->coutInfo();
	carObs->coutInfo();
	showScene();
	system("pause");
}

//display
void StraightFollow::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(GREEN);
	car0->showCar(BLACK);

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool StraightFollow::planning_process() {

	double dis = car0->pmidf->y - carObs->pmidr->y;//initial spacing of two car
	double delta_dis = dis - safedis;//init spaceing - target spacing
	double delta_speed_y = car0->speed_y - carObs->speed_y;// speed difference
	if (dis <= 0.0 || delta_dis <= 0.0 || delta_speed_y > 0.0)//only consider main car speed is greater than target car, need to decelerate.
	{
		return false;
	}

	car0->a_y = pow(delta_speed_y, 2) / delta_dis / 2.0;
	cout << "car0->a_y = " << car0->a_y << endl;

	while (car0->pmidr->y > 0.0)
	{
		car0->moveStraightStep();
		carObs->moveStraightStep();
		if (fabs(car0->speed_y - carObs->speed_y) > fabs(car0->a_y))//speed different is large enough
		{
			car0->speed_y += car0->a_y;
		}
		else
		{
			car0->speed_y = carObs->speed_y;
			car0->a_y = 0.0;
		}
		showScene();//display		
	}
	car0->coutInfo();
	return true;
}


//aaaaaaaaaaaaaaaaaaaa Crosswalk aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
StraightCrosswalk::StraightCrosswalk() {

	road0 = make_unique<RoadCrosswalk>();
	car0 = make_unique<CarNormal>(SWIDTH / 2.0, SHEIGHT - 70.0);
	car0->speed_y = -4.0;

	for (int i = 0; i < people_num; i++)
	{
		unique_ptr<Person> ps = make_unique<Person>(road0->right_boundary + 20 * (i * 3 + 1), road0->getMidLine());
		ps->speed = -2; //pedestrain speed
		people.push_back(move(ps));
	}

	car0->coutInfo();
	showScene();//display
	system("pause");
}

//check if crosswalk have pedestrian
bool StraightCrosswalk::peopleInCross() {

	for (auto& i : people)
	{
		if (i->p_center->x >= road0->left_boundary - i->r && i->p_center->x <= road0->right_boundary + i->r)
		{
			return true;
		}
	}
	return false;
}


//display the crosswalk scene
void StraightCrosswalk::showScene() {

	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	car0->showCar(BLACK);

	car0->moveStraightStep();
	car0->speed_y += car0->a_y;

	for (auto& i : people)
	{
		i->showPerson();
		i->personMove();
	}

	EndBatchDraw();
	delay(DELAYTIME);
}

//process
bool StraightCrosswalk::planning_process() {

	//entering to crosswalk, decelerate speed
	double dis = car0->pmidf->y - road0->getDownLine();
	car0->a_y = pow(car0->speed_y, 2) / 2.0 / dis;

	while (dis > 0.0)
	{
		dis = car0->pmidf->y - road0->getDownLine();
		if (!peopleInCross()) //if no pedestrian in crosswalk
		{
			if (car0->speed_y >= speedlimit_cross)// crosswalk limit speed
			{
				car0->a_y = 0.0;// become uniform speed
			}
		}
		else
		{
			if (dis <= 15.0)//stop the car at front of the crosswalk, prevent slow move or reverse action
			{
				car0->speed_y = 0.0;
				car0->a_y = 0.0;
				break;
			}
		}

		cout << "dis = " << dis << ", car speed_y = " << car0->speed_y << ", a_y = " << car0->a_y << endl;
		showScene();//
	}

	//pass crosswalk by limiting speed
	while (car0->pmidr->y > road0->getUpLine())//crosswalk intervel
	{
		if (!peopleInCross())//check if crosswalk have pedestrian crosswalk
		{
			if (car0->speed_y > speedlimit_cross)//if not reach the limit speed of crosswalk
			{
				car0->a_y = -0.05;//accelerate
			}
			else //reach the limit speed of crosswalk
			{
				car0->a_y = 0.0;//become uniform speed
			}
		}

		cout << "car speed_y = " << car0->speed_y << ", a_y = " << car0->a_y << endl;
		showScene();//display
	}

	//once pass the crosswalk, accelerating until reach the speed limit
	while (car0->pmidr->y > 0.0)
	{
		if (car0->speed_y > speedlimit)// if not reach the limit speed of crosswalk
		{
			car0->a_y = -0.05;//accelerate
		}
		else
		{
			car0->a_y = 0.0;//uniform speed
		}

		cout << "car speed_y = " << car0->speed_y << ", a_y = " << car0->a_y << endl;
		showScene();//display
	}
	
	return true;
}