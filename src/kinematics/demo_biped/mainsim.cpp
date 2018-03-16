#include "link.h"
#include "kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>
#include "pi.h"

using namespace std;

void SetDefaultAngle(Link* link)
{
  link[CC].q      = deg2rad(0.0);
  link[RLEG_J0].q = deg2rad(0.0);
  link[RLEG_J1].q = deg2rad(0.0);
  link[RLEG_J2].q = deg2rad(-30.0);
  link[RLEG_J3].q = deg2rad(60.0);
  link[RLEG_J4].q = deg2rad(-30.0);
  link[RLEG_J5].q = deg2rad(0.0);

  link[LLEG_J0].q = deg2rad(0.0);
  link[LLEG_J1].q = deg2rad(0.0);
  link[LLEG_J2].q = deg2rad(-30.0);
  link[LLEG_J3].q = deg2rad(60.0);
  link[LLEG_J4].q = deg2rad(-30.0);
  link[LLEG_J5].q = deg2rad(0.0);
}

void Now_State( Link *link, int mode )
{
	int EDF = RLEG_J5;
	cout << endl << "IK_mode :";
	if(	mode == 0) cout << "NR-method" << endl;
	else if(mode ==1) cout << "LM-method" << endl;
	cout << endl << "EDF_pos = [" << link[EDF].p(0) << ", " << link[EDF].p(1) << ", "<< link[EDF].p(2) << "]" << endl;
  cout << endl << "EDF_Rot = [" << rad2deg(link[EDF].R(0)) << ", " << rad2deg(link[EDF].R(1)) << ", "<< rad2deg(link[EDF].R(2)) << "]" << endl;
}

void print_usage()
{
	printf("------------\n");
	printf("x: 'f' or 'j'\n");
	printf("y: 'd' or 'k'\n");
	printf("z: 's' or 'l'\n");
	printf("------------\n");
	printf("------------\n");
	printf("roll	: 'r' or 'u'\n");
	printf("pitch	: 'e' or 'i'\n");
	printf("yaw	: 'w' or 'o'\n");
	printf("------------\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

void State( Link *link, int mode )
{
	Now_State( link, mode );
	print_usage();
}

int main(int argc, char *argv[])
{
	Link ulink[JOINT_NUM];
	Kinematics kinema(ulink);
	SetJointInfo( ulink );
	
	FILE *gp;
	int cmd = -2, mode = 0;
	double x = 0.0, y = 0.0, z = 0.0;
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
	double bx = 0.0, by = 0.0, bz = 0.0, i_r = 0.0, i_p = 0.0, i_y = 0.0;
	double pos_step = 0.005;
	double rot_step = 0.5;

	system( "stty -echo");
	gp = popen( "gnuplot","w");
	SetPlotConf( gp, 90, 360 );
	SetDefaultAngle(ulink);
	kinema.calcForwardKinematics(CC);
	Link Target_R = ulink[RLEG_J5];

	bx = Target_R.p(0);
	by = Target_R.p(1);
	bz = Target_R.p(2);
	
	i_r = rad2deg(Target_R.R(0));
	i_p = rad2deg(Target_R.R(1));
	i_y = rad2deg(Target_R.R(2));

	State( ulink, mode );

while(1)
	{
		if(kbhit())
		{
			InputKey( &cmd );
			if		 (cmd == 1) {  x += pos_step; }
			else if(cmd == 2) {  x -= pos_step; }
			else if(cmd == 3) {  y += pos_step; }
			else if(cmd == 4) {  y -= pos_step; }
			else if(cmd == 5) {  z += pos_step; }
			else if(cmd == 6) {  z -= pos_step; }
			else if(cmd == 7) {roll += rot_step; }
			else if(cmd == 8) {roll -= rot_step; }
			else if(cmd == 9) {pitch +=rot_step; }
			else if(cmd == 10){pitch -=rot_step; }
			else if(cmd == 11){ yaw += rot_step; }
			else if(cmd == 12){ yaw -= rot_step; }
			else if(cmd == 13){ mode = 0; }
			else if(cmd == 14){ mode = 1; }
			else if(cmd == -2)
			{ 
				SetDefaultAngle(ulink);
				kinema.calcForwardKinematics(CC);
				x = y = z = 0.0;
				roll = pitch = yaw = 0.0;
			}
			else if(cmd == -1) break;

			State( ulink, mode );
		}
		usleep(20000);
		
		Target_R.p(0) = bx + x;
		Target_R.p(1) = by + y;
		Target_R.p(2) = bz + z;
		Target_R.R = kinema.RotationFromRPY( deg2rad(roll), deg2rad(pitch), deg2rad(yaw) );

		if(mode == 0)kinema.calcInverseKinematics(RLEG_J5,Target_R);
		else if(mode ==1)kinema.calcLMInverseKinematics(RLEG_J5,Target_R);
		PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );

	}
	system( "stty echo");
	pclose(gp);
	return 0;
}

