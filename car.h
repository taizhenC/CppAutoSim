#pragma once
#include "planning_base.h"


//gear shift
enum Shift {

	m_D, //gear drive
	m_N, //gear neuthral
	m_R, //gear reverse
	m_P  //gear park
};

//turning
enum TurnDirection {
	TurnRight,
	TurnLeft,
};

//carbase
class CarBase {
public:
	virtual ~CarBase() = default;
	void initCar(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length);//initialize
	void updatePmidf();//update x,y,point of front center
	void updatePmidr();//update x,y,point of rear center 
	void updatePmid();//update x,y,point of center of turning circle
	void showCar(const COLORREF& color);// drawing car 
	void showCircle();//draw trajectory line
	void coutInfo();// print out the info

	void moveStraightStep();//single-frame straight motion
	void carTurnStep();//single-frame turning
	void carRotationStep();//

	void updateRinRout(const double& R);//update the 4 radius
	void updateTurnInfo(const int& turn_state, const double& R);//update the turing value

	void updateXYva();//update the x and y direction speed and acceleration
	void updateStraightInfo();// update the straight motion info
	void updateDriftRotInfo();//
	void updateDriftRotRevInfo(const Point& center);//

public:
	double car_width = 80.0; //car width
	double car_length = 160.0; // car length

	unique_ptr<Point> plf; //left front
	unique_ptr<Point> plr; //left rear
	unique_ptr<Point> prf; //right front
	unique_ptr<Point> prr; //right rear
	unique_ptr<Point> p_center; //turning center point

	unique_ptr<Point> pmidf; //front center point
	unique_ptr<Point> pmidr; //rear center point
	unique_ptr<Point> pmid; //center of turning circle

	double Rmin = 100.0; // minimum truing radius
	double Rof = 0.0; // radius of right front point from pmid
	double Ror = 0.0; // radius of right rear point from pmid
	double Rif = 0.0; // radius of left front point from pmid
	double Rir = 0.0; // radius of left rear point from pmid

	double R0;// center of the car
	double theta0;//atan(car_length / car_width)

	double speed = 0.0; //total speed, positive forward, negative backward
	double speed_x = 0.0; //speed of x-axis,positive leftward, negative rightward
	double speed_y = 0.0; //speed of y-axis,positive downward, negative upward

	double a = 0.0;// total acceleration, + -, + mean accelerate, - mean decelerate
	double a_x = 0.0;//acclerate of x-axis, + mean accelerate rightward, - mean accelerate leftward
	double a_y = 0.0;//accelerate of y-axis, + accelerate downward, - mean accelerate upward

	double delta_theta = 0.0; //angular speed, positive counterclock, negative clockwise
	double delta_theta_rot = 0.0; //rotational angular velocity, positive counterclock, negative clockwise
	double heading_theta = 0.0; //heading angle, 0 mean car is directly upward, positive right yaw, negative left yaw

	int Gear = m_P; //gear of the car, default to park gear
};

//normal car
class CarNormal : public CarBase {
public:
	CarNormal(const double& pos_x, const double& pos_y, const double& heading = 0.0, const double& width = 80.0, const double& length = 160.0);
};
