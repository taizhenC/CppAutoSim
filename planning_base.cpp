#include "planning_base.h"

//point
Point::Point(const double& p_x, const double& p_y, const double& p_theta, const double& p_R) : x(p_x), y(p_y), thetaP(p_theta), Rp(p_R) {

}

//dot drawing
void Point::showPoint() {

	setfillcolor(BLACK);
	solidcircle(x, y, r);
}

//movment of the point
void Point::pointMove(const double& speed_x, const double& speed_y) {

	x += speed_x;
	y += speed_y;
}

//point rotating, turn_speed is angular speed
void Point::pointTurn(const Point& center, const double& turn_speed) {

	thetaP += turn_speed; // turn_speed>0, mean the center to do counterclock, <0 mean to do clockwise
	x = Rp * cos(thetaP) + center.x;
	y = -Rp * sin(thetaP) + center.y; //base on easyx y-axis, so add negative.
}

//return distance
double Point::distanceTo(const Point& p) const {

	return hypot(x - p.x, y - p.y);
}

//relative p angle
double Point::thetaTo(const Point& p) const {

	if (x >= p.x && y == p.y)//+x-axis, overlap with p
	{
		return 0.0;
	}
	else if (x < p.x && y == p.y)//- x-axis
	{
		return PI;
	}
	else if (x == p.x && y > p.y)//- y-axis
	{
		return -PI / 2.0;
	}
	else if (x == p.x && y < p.y)//+y-axis
	{
		return PI / 2.0;
	}
	else if (x > p.x)//first, four quadrants
	{
		return -atan((y - p.y) / (x - p.x));//base on easyx coordinate, take -
	}
	else if (x < p.x)//second, third quadrants
	{
		return PI - atan((y - p.y) / (x - p.x));
	}

	return 0.0;
}
////////////////////////// vector ///////////////////////
// use two value to form vector, flag use to seprate another function
Vec2d::Vec2d(const double& new_x, const double& new_y, const bool& flag) : x(new_x), y(new_y) {


}

Vec2d::Vec2d(const Point& p_start, const Point& p_end)// use two point form vector
{
	x = p_end.x - p_start.x;
	y = -(p_end.y - p_start.y);//vector switch to y-axis coordinate
}

//use length and direction to form vector
Vec2d::Vec2d(const double& length, const double& angle) {

	x = length * cos(angle);
	y = length * sin(angle);//vector switch to y-axis coordinate
}

//mod
double Vec2d::length() {

	return hypot(x, y);
}

//corss product
double Vec2d::crossProd(const Vec2d& other) const {

	return x * other.y - y * other.x;
}

//dot product
double Vec2d::innerProd(const Vec2d& other) const {

	return x * other.x + y * other.y;
}

///////////////////////////////////////////////////////////////////////////global function
//delaying the motion of screen, ms,helper function
void delay(const int& time) {

	clock_t now = clock();
	while (clock() - now < time) {

	}
}

//angle fix to [-pi,pi) 
double normalizeAngle(const double& theta) {

	double theta_new = fmod(theta + PI, 2.0 * PI);//angle mod with 2pi
	if (theta_new < 0.0) {

		theta_new += (2.0 * PI);//if less than 0, add 2pi
	}
	return theta_new - PI;//subtract pi
}

//angle error fix
void correctAngleError(double& target_theta, const double& delta_theta) {

	if (delta_theta > 0.0)
	{
		if (target_theta > 0)//couterclockwise
		{
			target_theta -= delta_theta;
		}
		else if (target_theta < 0)//clockwise
		{
			target_theta += delta_theta;
		}
	}
}

//distance between point and line
double disPointToLine(const Point& p, const Point& p_start, const Point& p_end) {

	Vec2d line(p_start, p_end);
	Vec2d line_p(p_start, p);
	if (line.length() == 0.0)
	{
		return line_p.length();
	}
	return fabs(line.crossProd(line_p)) / line.length();
}