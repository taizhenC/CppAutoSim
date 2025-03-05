#pragma once
#include <iostream>
#include <graphics.h>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <memory>
using namespace std;
//global variable

constexpr auto SWIDTH = 1200.0;//screen width
constexpr auto SHEIGHT = 1200.0;//screen height
constexpr auto PI = 3.14159265358979323846;
constexpr auto SHOWCIRCLE = false;//draw trjectory
constexpr auto DELAYTIME = 20; //time between one frame, ms
constexpr auto CHANGETIME = 1000; //change gear time, ms

//dot
class Point{
public:
	Point() = default;
	Point(const double& p_x, const double& p_y, const double& p_theta = 0.0, const double& p_R = 0.0);
	void showPoint(); //paint dot
	void pointMove(const double& speed_x, const double& speed_y);//motion of dot
	void pointTurn(const Point& center, const double& turn_speed); //point around center to rotate, turn_speed is angular speed
	double distanceTo(const Point& p) const;//distance
	double thetaTo(const Point& p) const;// relative p angle
public:
	double x;
	double y;
	double thetaP = 0.0;//angle
	double Rp = 0.0;//rotation radius
	int r = 5; //radius
};

//vector2d
class Vec2d {

public:
	Vec2d() = default;
	Vec2d(const double& new_x, const double& new_y, const bool& flag);//use two value to form vector, flag use to seprate another function
	Vec2d(const Point& p_start, const Point& p_end);//use two point form vector
	Vec2d(const double& length, const double& angle);//use length and direction to form vector
	double length();//length
	double crossProd(const Vec2d& other) const;//cross product
	double innerProd(const Vec2d& other) const;//dot product

public:
	double x;
	double y;
};



//global function,delay, ms
void delay(const int& time);
//angle fix to [-pi,pi) 
double normalizeAngle(const double& theta); 
//angle error fix
void correctAngleError(double& target_theta, const double& delta_theta); 
//distance between point and line
double disPointToLine(const Point& p, const Point& p_start, const Point& p_end);