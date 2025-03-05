#pragma once
#include "planning_base.h"
#include "car.h"


//obstacle
class Cone {

public:
	Cone() = default;
	Cone(const double& pos_x, const double& pos_y, const double& R = 20.0);
	void showCone();//drawing

public:
	unique_ptr<Point> p_center;//center point
	double r = 20.0;//radius
};

//pedestrian
class Person
{
public:
	Person() = default;
	Person(const double& pos_x, const double& pos_y);
	void personMove();//pedestrian move
	void showPerson();//display

public:
	unique_ptr<Point> p_center;//center point
	double r = 20.0;//radius
	double speed = 0.0;//pdestrian speed
};