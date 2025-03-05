#include "traffic.h"

//parameter constructor
Cone::Cone(const double& pos_x, const double& pos_y, const double& R) : r(R) {

	p_center = make_unique<Point>(pos_x, pos_y);
}

//drawing cone
void Cone::showCone() {
	setfillcolor(RGB(255, 127, 0));//orange
	solidcircle(p_center->x, p_center->y, r);
}

//pedstrain
Person::Person(const double& pos_x, const double& pos_y) {

	p_center = make_unique<Point>(pos_x, pos_y);
}

//pedstrain moving
void Person::personMove() {

	p_center->x += speed;
}

//display
void Person::showPerson() {

	setfillcolor(GREEN);
	fillcircle(p_center->x, p_center->y, r);
}