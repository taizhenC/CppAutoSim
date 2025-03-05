#include "car.h"


//carbase
void CarBase::initCar(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length) {

	car_width = width;
	car_length = length;
	heading_theta = heading;

	R0 = hypot(car_width / 2.0, car_length / 2.0);
	theta0 = atan(car_length / car_width);

	//the four vertex
	pmid = make_unique<Point>(pos_x, pos_y);//center of turning circle
	plf = make_unique<Point>(pmid->x - car_width / 2.0, pmid->y - car_length / 2.0, PI - theta0, R0);
	prf = make_unique<Point>(pmid->x + car_width / 2.0, pmid->y - car_length / 2.0, theta0, R0);
	plr = make_unique<Point>(pmid->x - car_width / 2.0, pmid->y + car_length / 2.0, PI + theta0, R0);
	prr = make_unique<Point>(pmid->x + car_width / 2.0, pmid->y + car_length / 2.0, -theta0, R0);

	//rotate of four vertex
	plf->pointTurn(*pmid, heading_theta);
	prf->pointTurn(*pmid, heading_theta);
	plr->pointTurn(*pmid, heading_theta);
	prr->pointTurn(*pmid, heading_theta);

	//update the front and rear mid point
	updatePmidf();
	updatePmidr();
}

//update the front mid point
void CarBase::updatePmidf() {

	double x = (plf->x + prf->x) / 2.0;
	double y = (plf->y + prf->y) / 2.0;
	if (pmidf) {
		pmidf->x = x;
		pmidf->y = y;
	}
	else {
		pmidf = make_unique<Point>(x, y);
	}
}

//update the rear mid point
void CarBase::updatePmidr() {

	double x = (plr->x + prr->x) / 2.0;
	double y = (plr->y + prr->y) / 2.0;
	if (pmidr) {

		pmidr->x = x;
		pmidr->y = y;
	}
	else {

		pmidr = make_unique<Point>(x, y);
	}
}

//update the center of turning circle
void CarBase::updatePmid() {

	double x = (plr->x + prr->x) / 2.0;
	double y = (plr->y + prr->y) / 2.0;
	if (pmidr) {

		pmidr->x = x;
		pmidr->y = y;
	}
	else {

		pmidr = make_unique<Point>(x, y);
	}
}

//draw the car
void CarBase::showCar(const COLORREF& color) {

	setlinestyle(PS_SOLID, 4);	// setting the type of line, solid line
	setlinecolor(color);
	line(plf->x, plf->y, prf->x, prf->y);
	line(prf->x, prf->y, prr->x, prr->y);
	line(prr->x, prr->y, plr->x, plr->y);
	line(plr->x, plr->y, plf->x, plf->y);
}

//draw trajectory line
void CarBase::showCircle() {

	setlinestyle(PS_DOT, 2); //dash line
	setlinecolor(MAGENTA);
	circle(p_center->x, p_center->y, Rof);
	circle(p_center->x, p_center->y, Ror);
	circle(p_center->x, p_center->y, Rif);
	circle(p_center->x, p_center->y, Rir);
}

//display the info
void CarBase::coutInfo()
{
	cout << "pmidr->x= " << pmidr->x << " pmidr->y= " << pmidr->y << " pmidr->Rp= " << pmidr->Rp << " pmidr->thetaP= " << pmidr->thetaP << endl;
	cout << "pmidf->x= " << pmidf->x << " pmidf->y= " << pmidf->y << " pmidf->Rp= " << pmidf->Rp << " pmidf->thetaP= " << pmidf->thetaP << endl;
	cout << "pmid->x= " << pmid->x << " pmid->y= " << pmid->y << " pmid->Rp= " << pmid->Rp << " pmid->thetaP= " << pmid->thetaP << endl;
	cout << "plf->x= " << plf->x << " plf->y= " << plf->y << " plf->Rp= " << plf->Rp << " plf->thetaP= " << plf->thetaP << endl;
	cout << "prf->x= " << prf->x << " prf->y= " << prf->y << " prf->Rp= " << prf->Rp << " prf->thetaP= " << prf->thetaP << endl;
	cout << "plr->x= " << plr->x << " plr->y= " << plr->y << " plr->Rp= " << plr->Rp << " plr->thetaP= " << plr->thetaP << endl;
	cout << "prr->x= " << prr->x << " prr->y= " << prr->y << " prr->Rp= " << prr->Rp << " prr->thetaP= " << prr->thetaP << endl;
	cout << "speed = " << speed << ", a = " << a << ", delta_theta = " << delta_theta / PI << ", delta_theta_rot = " << delta_theta_rot / PI << ", heading_theta = " << heading_theta / PI << endl;
}

//single-frame straight motion
void CarBase::moveStraightStep()
{
	plf->pointMove(speed_x, speed_y);
	prf->pointMove(speed_x, speed_y);
	plr->pointMove(speed_x, speed_y);
	prr->pointMove(speed_x, speed_y);
	pmidf->pointMove(speed_x, speed_y);
	pmidr->pointMove(speed_x, speed_y);
	pmid->pointMove(speed_x, speed_y);
}

//single-frame turning
void CarBase::carTurnStep()
{
	pmidr->pointTurn(*p_center, delta_theta);
	plf->pointTurn(*p_center, delta_theta);
	prf->pointTurn(*p_center, delta_theta);
	plr->pointTurn(*p_center, delta_theta);
	prr->pointTurn(*p_center, delta_theta);
	heading_theta += delta_theta;
}

//update 4 radius
void CarBase::updateRinRout(const double& R) {

	Ror = R + car_width / 2.0;
	Rir = R - car_width / 2.0;
	Rof = hypot(Ror, car_length);
	Rif = hypot(Rir, car_length); 
}




//update turn info
void CarBase::updateTurnInfo(const int& turn_state, const double& R)//����ת����Ϣ
{
	double x = 0.0;
	double y = 0.0;
	updateRinRout(R);//update 4 radius
	cout << "Rof = " << Rof << ", Rif = " << Rif << ", Ror = " << Ror << ", Rir = " << Rir << endl;

	if (turn_state == TurnDirection::TurnRight)//right turn
	{
		//calculate the turn center coordinate
		x = pmidr->x + R * cos(heading_theta);
		y = pmidr->y - R * sin(heading_theta);

		//update angle and radius of car, 5 point
		pmidr->thetaP = heading_theta + PI;
		pmidr->Rp = R;

		plr->thetaP = pmidr->thetaP;
		plr->Rp = Ror;

		prr->thetaP = pmidr->thetaP;
		prr->Rp = Rir;

		plf->thetaP = pmidr->thetaP - atan(car_length / Ror);
		plf->Rp = Rof;

		prf->thetaP = pmidr->thetaP - atan(car_length / Rir);
		prf->Rp = Rif;
	}
	else//left turn
	{
		//calculate the center coordinate
		x = pmidr->x - R * cos(heading_theta);
		y = pmidr->y + R * sin(heading_theta);

		//update the radius and angle of car, 5 point
		pmidr->thetaP = heading_theta;
		pmidr->Rp = R;

		plr->thetaP = pmidr->thetaP;
		plr->Rp = Rir;

		prr->thetaP = pmidr->thetaP;
		prr->Rp = Ror;

		plf->thetaP = pmidr->thetaP + atan(car_length / Rir);
		plf->Rp = Rif;

		prf->thetaP = pmidr->thetaP + atan(car_length / Ror);
		prf->Rp = Rof;
	}
	cout << "center_turn.x= " << x << ", center_turn.y= " << y << endl;

	//update turn center
	if (p_center)
	{
		p_center->x = x;
		p_center->y = y;
	}
	else
	{
		p_center = make_unique<Point>(x, y);
	}
}


//update the x and y direction speed and accerlation
void CarBase::updateXYva()
{
	speed_x = speed * sin(heading_theta);
	speed_y = speed * cos(heading_theta);
	a_x = a * sin(heading_theta);
	a_y = a * cos(heading_theta);
	cout << "speed_x = " << speed_x << ", speed_y = " << speed_y << ", a_x = " << a_x << ", a_y = " << a_y << endl;
}


//update the straight motion info
void CarBase::updateStraightInfo()
{
	updatePmidr();
	updatePmidf();
	updatePmid();
	updateXYva();
	p_center.reset();

	Ror = 0.0;
	Rir = 0.0;
	Rof = 0.0;
	Rif = 0.0;

	pmidr->thetaP = 0.0;
	pmidr->Rp = 0.0;

	pmidf->thetaP = 0.0;
	pmidf->Rp = 0.0;

	pmid->thetaP = 0.0;
	pmid->Rp = 0.0;

	plr->thetaP = 0.0;
	plr->Rp = 0.0;

	prr->thetaP = 0.0;
	prr->Rp = 0.0;

	plf->thetaP = 0.0;
	plf->Rp = 0.0;

	prf->thetaP = 0.0;
	prf->Rp = 0.0;

	delta_theta = 0.0;
	delta_theta_rot = 0.0;
}



//normal car
CarNormal::CarNormal(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length) //һ�㳵
{
	initCar(pos_x, pos_y, heading, width, length);
}