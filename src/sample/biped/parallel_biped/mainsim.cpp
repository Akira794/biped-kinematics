#include "Link.h"
#include "Kinematics.h"
#include "plot.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>
#include "../util/func.h"

using namespace std;

void SetDefaultAngle(Link* link)
{
	link[LR2].q =  deg2rad(0.0);
	link[LP4].q = -deg2rad(50.0) + deg2rad(0.0);;
	link[LP3].q =  deg2rad(50.0);
	link[LP2].q = -deg2rad(-50.0);
	link[LP1].q =  deg2rad(-50.0);
	link[LR1].q =  deg2rad(0.0);
	link[LY ].q =  deg2rad(0.0);

	link[RR2].q =  deg2rad(0.0);
	link[RP4].q = -deg2rad(50.0) + deg2rad(0.0);;
	link[RP3].q =  deg2rad(50.0);
	link[RP2].q = -deg2rad(-50.0);
	link[RP1].q =  deg2rad(-50.0);
	link[RR1].q =  deg2rad(0.0);
	link[RY ].q =  deg2rad(0.0);
}

void Now_State( Link *link, int mode, int foot )
{
	int EDF = 0;
	if(foot == 1) EDF = RF;
	else if( foot == 2) EDF = LF;

	cout << endl << "IK_mode :";
	if(	mode == 0) cout << "NR-method" << endl;
	else if(mode ==1) cout << "LM-method" << endl;
	cout << endl << link[EDF].joint_name << " pos = [" << link[EDF].p(0) << ", " << link[EDF].p(1) << ", "<< link[EDF].p(2) << "]" << endl;
	cout << endl << link[EDF].joint_name << " Rot = [" << rad2deg(link[EDF].R(0)) << ", " << rad2deg(link[EDF].R(1)) << ", "<< rad2deg(link[EDF].R(2)) << "]" << endl;
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
	printf("------------\n");
	printf("Foot : 'z':right 'm':left \n");
	printf("------------\n");
	printf("NR-IK: 'c'  \n");
	printf("LM-IK: 'n'  \n");
	printf("Reset : 'x' \n");
	printf("------------\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

void State( Link *link, int mode, int foot )
{
	Now_State( link, mode, foot );
	print_usage();
}

int main(int argc, char *argv[])
{
	Link ulink[LINK_NUM];
	Kinematics kinema(ulink);
	SetJointInfo( ulink );
	int target_link = 0;
	FILE *gp;
	int cmd = -2, mode = 0, foot = 1;
	double x = 0.0, y = 0.0, z = 0.0;

	double lx, ly, lz, rx, ry, rz, l_tr, l_tp, l_ty, r_tr, r_tp, r_ty;

	double roll  = 0.0, pitch = 0.0, yaw   = 0.0;
	double bx = 0.0, by = 0.0, bz = 0.0, broll = 0.0, bpitch = 0.0, byaw = 0.0;
	double pos_step = 0.005;
	double rot_step = 0.5;

	system( "stty -echo");
	gp = popen( "gnuplot","w");
	SetPlotConf( gp, 74, 54 );
	SetDefaultAngle(ulink);
	kinema.calcForwardKinematics(BASE);	
	for(int i =0; i < LINK_NUM; i++)
	{
		cout << rad2deg(ulink[i].q) << endl;
	}
	PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );

	target_link = RR2;

	Link Target = ulink[target_link];
	bx = Target.p(0);
	by = Target.p(1);
	bz = Target.p(2);
	
	State( ulink, mode, foot );

	while(1)
	{
		if(kbhit())
		{
			InputKey( &cmd );
			if		 (cmd == 1) x += pos_step;
			else if(cmd == 2) x -= pos_step;
			else if(cmd == 3) y += pos_step;
			else if(cmd == 4) y -= pos_step;
			else if(cmd == 5) z += pos_step;
			else if(cmd == 6) z -= pos_step;
			else if(cmd == 7) roll += rot_step;
			else if(cmd == 8) roll -= rot_step;
			else if(cmd == 9) pitch +=rot_step;
			else if(cmd == 10) pitch -=rot_step;
			else if(cmd == 11) yaw += rot_step;
			else if(cmd == 12) yaw -= rot_step;
			else if(cmd == 13) mode = 0;
			else if(cmd == 14) mode = 1;
			else if(cmd == 15)
			{ foot = 1;

				lx = x; ly = y; lz = z; 
				l_tr = roll; l_tp = pitch; l_ty = yaw;

				x = rx; y = ry; z = rz;
				roll = r_tr; pitch = r_tp; yaw = r_ty;

				target_link = RR2; 
				by = ulink[target_link].p(1);
				Target = ulink[target_link];
			}
			else if(cmd == 16)
			{ foot = 2;

				rx = x; ry = y; rz = z;
				r_tr = roll; r_tp = pitch; r_ty = yaw;

				x = lx; y = ly; z = lz;
				roll = l_tr; pitch= l_tp;  yaw = l_ty;

				target_link = LR2;
				by = ulink[target_link].p(1);
				Target = ulink[target_link];
			}
			else if(cmd == -1) break;
			else if(cmd == -2)
			{ 
				SetDefaultAngle(ulink);
				kinema.calcForwardKinematics(BASE);
				x = y = z = lx = ly = lz = rx = ry = rz = 0.0;
				roll = pitch = yaw = l_tr = l_tp = l_ty = r_tr = r_tp = r_ty = 0.0;
				Target = ulink[RR2];
				Target = ulink[LR2];
			}
			State( ulink, mode, foot );
		}
		usleep(10000);

		Target.p << bx + x, by +y, bz + z;
		Target.R = kinema.computeMatrixFromAngles( deg2rad(broll + roll), deg2rad(bpitch + pitch), deg2rad(byaw + yaw) );

		if(mode == 0)kinema.calcInverseKinematics(target_link,Target);
		else if(mode ==1)kinema.calcLMInverseKinematics(target_link,Target);

		PlotLeg( gp, ulink, 0.0, 0.0, -0.0, -0.0 );

	}

	system( "stty echo");
	pclose(gp);
	return 0;
}

