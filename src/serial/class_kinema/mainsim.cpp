#include "link.h"
#include "kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>

using namespace std;

void SetDefaultAngle(Link* link)
{
  link[CC].q      = Deg2Rad(0.0);
  link[RLEG_J0].q = Deg2Rad(0.0);
  link[RLEG_J1].q = Deg2Rad(0.0);
  link[RLEG_J2].q = Deg2Rad(-30.0);
  link[RLEG_J3].q = Deg2Rad(60.0);
  link[RLEG_J4].q = Deg2Rad(-30.0);
  link[RLEG_J5].q = Deg2Rad(0.0);

  link[LLEG_J0].q = Deg2Rad(0.0);
  link[LLEG_J1].q = Deg2Rad(0.0);
  link[LLEG_J2].q = Deg2Rad(-30.0);
  link[LLEG_J3].q = Deg2Rad(60.0);
  link[LLEG_J4].q = Deg2Rad(-30.0);
  link[LLEG_J5].q = Deg2Rad(0.0);
}

int main(int argc, char *argv[])
{
	Link ulink[JOINT_NUM];
	Kinematics kinema(ulink);
	SetJointInfo( ulink );
	
	FILE *gp;
	int cmd = 0;
	double rx = 0.0, ry = 0.0, rz = 0.0;
	double bx = 0.0, by = 0.0, bz = 0.0;

	system( "stty -echo");
	gp = popen( "gnuplot","w");
	SetPlotConf( gp, 56, 47 );
	SetDefaultAngle(ulink);
	kinema.CalcForwardKinematics(CC);
	Link Target_R = ulink[RLEG_J5];
	Link Target_L = ulink[LLEG_J5];

	bx = Target_L.p(0);
	by = Target_L.p(1);
	bz = Target_L.p(2);

	cout << Target_L.p << endl;

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
				SetDefaultAngle(ulink);
				kinema.CalcForwardKinematics(CC);
				rx = ry = rz = 0.0;
		}
		usleep(20000);
		
		Target_L.p(0) = bx + rx;
		Target_L.p(1) = by + ry;
		Target_L.p(2) = bz + rz;

		kinema.CalcIK_LM(LLEG_J5,Target_L);
		PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );

	}
	system( "stty echo");
	pclose(gp);
	return 0;
}

