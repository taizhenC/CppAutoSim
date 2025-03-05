#pragma once
#include "traffic.h"


class RoadBase {

public:
	virtual ~RoadBase() = default;
	virtual void showRoad() = 0; //drawing road

	virtual double getUpLine() { return 0.0;}
	virtual double getMidLine() { return 0.0; }
	virtual double getDownLine() { return 0.0; }
public:
	double Rwidth = 200.0;//road width
	double up_boundary = 0.0;//up boundary
	double down_boundary = 0.0;//down boundary
	double left_boundary = 0.0;//left boundary
	double right_boundary = 0.0;//right boundary
};

//general road
class RoadNormal : public RoadBase {

public:
	RoadNormal(const double& r_width = 200.0);
	void showRoad() override;//drawing road
};

//double two-lane road
class RoadDoubleLane : public RoadBase {

public:
	RoadDoubleLane(const double& r_width = 200.0);
	void showRoad() override;//display road
};


class RoadCrosswalk : public RoadBase {
public:
	RoadCrosswalk(const double& r_width = 200.0);
	void showRoad() override;//display
	double getUpLine() { return this->up_line; }//return crosswalk top boundary
	double getMidLine() { return this->mid_line; }//return crosswalk center line
	double getDownLine() { return this->down_line; }//return crosswalk bottom boundary

public:
	double up_line = 0.0;//crosswalk top boundary
	double mid_line = 0.0;//crosswalk center line
	double down_line = 0.0;//crosswalk bottom boundary
	double disRec = 20.0;//spacing
};
