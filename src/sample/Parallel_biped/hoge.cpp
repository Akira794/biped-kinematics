#include "Kinematics.h"
#include "plot.h"
#include <iostream>
#include "../util/func.h"

using namespace std;

void SetDefaultAngle(Link* link)
{
	link[LR2].q =  deg2rad(0.0);
	link[LP4].q = -deg2rad(10.0) + deg2rad(0.0);;
	link[LP3].q =  deg2rad(10.0);
	link[LP2].q = -deg2rad(-10.0);
	link[LP1].q =  deg2rad(-10.0);
	link[LR1].q =  deg2rad(0.0);
	link[LY ].q =  deg2rad(0.0);

	link[RR2].q = deg2rad(0.0);
	link[RP4].q = deg2rad(10.0) + deg2rad(0.0);;
	link[RP3].q = deg2rad(10.0);
	link[RP2].q = deg2rad(-10.0);
	link[RP1].q = deg2rad(-10.0);
	link[RR1].q = deg2rad(0.0);
	link[RY ].q = deg2rad(0.0);
}

int main(int argc, char *argv[])
{
	Link ulink[LINK_NUM];
	Kinematics kinema(ulink);
	SetJointInfo( ulink );
	FILE *gp;

	gp = popen("gnuplot -persist", "w");
	SetPlotConf(gp, 74,54);
	SetDefaultAngle(ulink);
	kinema.calcForwardKinematics(BASE);
	PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );
	return 0;
}
