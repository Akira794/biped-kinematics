#include "link.h"
#include "kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>

using namespace std;

int main(int argc, char *argv[])
{
	Link ulink[JOINT_NUM], Target_R, Target_L;
	FILE *gp;

	LinkInit( ulink );
	LinkDefaultInit( ulink );
	gp = popen( "gnuplot -persist","w");
	SetPlotConf( gp, 54, 53 );

		SetFootConf( &Target_R, 0.0, 0.0 - 0.044, -0.25, 0.0, 0.0, 0.0);
		SetFootConf( &Target_L, 0.0, 0.0 + 0.044, -0.20, 0.0, 0.0, 0.0);

		CalcInverseKinematics( ulink, RLEG_J5, Target_R);
		CalcInverseKinematics( ulink, LLEG_J5, Target_L);

		PlotLeg( gp, ulink, 0.0, 0.0, -0.2, -0.0 );

	pclose(gp);
	LinkDestroy( ulink );

	return 0;
}

