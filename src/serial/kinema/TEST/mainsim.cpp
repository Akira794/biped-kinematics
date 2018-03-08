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
	int cmd = 0;
	double rx = 0.0, ry = 0.0, rz = 0.0;

	system( "stty -echo ");
	LinkInit( ulink );
	LinkDefaultInit( ulink );
	CalcForwardKinematics( ulink, CC);

	gp = popen( "gnuplot","w");
	SetPlotConf( gp, 56, 47 );

	while(1)
	{
		if(kbhit())
		{
			InputKey( &cmd );
			if		 (cmd == 1) ry += 0.005;
			else if(cmd == 2) ry -= 0.005;
			else if(cmd == 3) rx -= 0.005;
			else if(cmd == 4) rx += 0.005;
			else if(cmd == 5) rz += 0.005;
			else if(cmd == 6) rz -= 0.005;
		}
		if(cmd == -1) break;
		else if(cmd == 10)
		{
				LinkInit(ulink);
				LinkDefaultInit(ulink);
				CalcForwardKinematics( ulink, CC);
				rx = ry = rz = 0.0;
				SetFootConf( &Target_R, 0.0, -0.044, -0.25, 0.0, 0.0, 0.0);
		}
		usleep(20000);

		SetFootConf( &Target_R, rx, ry - 0.044, -0.25 + rz, 0.0, 0.0, 0.0);
		SetFootConf( &Target_L, 0.0, 0.0 + 0.044, -0.25, 0.0, 0.0, 0.0);

		InverseKinematicsAll( ulink, Target_R, Target_L);
		PlotLeg( gp, ulink, 0.0, 0.0, -0.2, -0.0 );

	}
	system( "stty echo");
	pclose(gp);
	LinkDestroy( ulink );

	return 0;
}

