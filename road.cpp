#include "road.h"

//general road
RoadNormal::RoadNormal(const double& r_width)//һ���·
{
	Rwidth = r_width;
	left_boundary = SWIDTH / 2.0 - Rwidth;
	right_boundary = SWIDTH / 2.0 + Rwidth;
}


//showing road
void RoadNormal::showRoad()
{
	setlinestyle(PS_SOLID, 4);
	setlinecolor(BLACK);
	line(left_boundary, 0.0, left_boundary, SHEIGHT);
	line(right_boundary, 0.0, right_boundary, SHEIGHT);
}

//double lane
RoadDoubleLane::RoadDoubleLane(const double& r_width) {

	Rwidth = r_width;
	left_boundary = SWIDTH / 2.0 - Rwidth;
	right_boundary = SWIDTH / 2.0 + Rwidth;
}

void RoadDoubleLane::showRoad() {

	setlinestyle(PS_SOLID, 4);
	setlinecolor(BLACK);
	line(left_boundary, 0.0, left_boundary, SHEIGHT);
	line(right_boundary, 0.0, right_boundary, SHEIGHT);

	setlinestyle(PS_DASH, 4);
	line(SWIDTH / 2.0, 0.0, SWIDTH / 2.0, SHEIGHT);
}

//crosswalk road
RoadCrosswalk::RoadCrosswalk(const double& r_width) {

	Rwidth = r_width;
	left_boundary = SWIDTH / 2.0 - Rwidth;
	right_boundary = SWIDTH / 2.0 + Rwidth;
	mid_line = SHEIGHT / 2.0 - 200.0;
	up_line = mid_line - Rwidth / 2.0;
	down_line = mid_line + Rwidth / 2.0;
}

//display for crosswalk road
void RoadCrosswalk::showRoad() {

	setlinestyle(PS_SOLID, 4);
	setlinecolor(BLACK);

	line(left_boundary, 0.0, left_boundary, SHEIGHT);
	line(right_boundary, 0.0, right_boundary, SHEIGHT);
	line(left_boundary, up_line, right_boundary, up_line);
	line(left_boundary, down_line, right_boundary, down_line);

	int i = 0;
	while (true)
	{
		double up_bound = up_line + disRec;
		double down_bound = down_line - disRec;
		double left_bound = left_boundary + disRec * (1 + 2 * i);
		double right_bound = left_boundary + disRec * 2 * (i + 1);
		if (right_bound >= right_boundary)
		{
			break;
		}
		rectangle(left_bound, up_bound, right_bound, down_bound);
		i++;
	}
}