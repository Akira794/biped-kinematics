#include "link.h"
#include "kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>

using namespace std;

void SetDefaultAngle(Link* link)
{
  link[BASE].q = Deg2Rad(0.0);
  link[ARM0].q = Deg2Rad(  0.0);
  link[ARM1].q = Deg2Rad(-30.0);
  link[ARM2].q = Deg2Rad( 60.0);
  link[ARM3].q = Deg2Rad(  0.0);
  link[ARM4].q = Deg2Rad( 30.0);
  link[ARM5].q = Deg2Rad(  0.0);
}

int main(int argc, char *argv[])
{
	Link ulink[JOINT_NUM];
	Kinematics kinema(ulink);
	SetJointInfo( ulink );
	
	FILE *gp;
	int cmd = 0, mode = 0;
	double rx = 0.0, ry = 0.0, rz = 0.0;
	double bx = 0.0, by = 0.0, bz = 0.0;

	system( "stty -echo");
	gp = popen( "gnuplot","w");
	SetPlotConf( gp, 56, 47 );
	SetDefaultAngle(ulink);
	kinema.CalcForwardKinematics(BASE);
	Link Mani = ulink[ARM5];

	bx = Mani.p(0);
	by = Mani.p(1);
	bz = Mani.p(2);

	cout << Mani.p << endl;

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
			else if(cmd == 7) mode = 0;
			else if(cmd == 8) mode = 1;
		}
		if(cmd == -1) break;
		else if(cmd == 10)
		{
				SetDefaultAngle(ulink);
				kinema.CalcForwardKinematics(BASE);
				rx = ry = rz = 0.0;
		}
		usleep(20000);
		
		Mani.p(0) = bx + rx;
		Mani.p(1) = by + ry;
		Mani.p(2) = bz + rz;

//		kinema.CalcInverseKinematics(ARM5,Mani);
		kinema.CalcIK_LM(ARM5,Mani);

		PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );

	}
	system( "stty echo");
	pclose(gp);
	return 0;
}

