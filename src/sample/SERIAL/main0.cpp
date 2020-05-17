#include "Kinematics.h"
#include "cmd.h"
#include <cstdio>
#include <cstdlib>
#include "../util/func.h"

using namespace std;

void SetDefaultAngle(Link*);
void Now_State( Link*, int, int );
void print_usage(void);
void State(Link*, int, int );
void SetPlotConf(FILE*, int, int );
double _elm( Link*, int, int );
void PlotLeg( FILE*, Link*, double, double, double, double );

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
	kinema.calcForwardKinematics(CC);

	target_link = JY3;

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
				target_link = JY3;
				//by = ulink[target_link].p(1);
				Target = ulink[target_link];
			}

			else if(cmd == -1) break;
			else if(cmd == -2)
			{
				SetDefaultAngle(ulink);
				kinema.calcForwardKinematics(CC);
				x = y = z = lx = ly = lz = 0.0;
				roll = pitch = yaw = l_tr = l_tp = l_ty = 0.0;
				Target = ulink[JY3];
				Target = ulink[JY3];
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

//

void SetDefaultAngle(Link* link)
{
	link[CC].q  = 0.0;
	link[JY1].q = deg2rad(0);
	link[JP1].q = deg2rad(25);
	link[JP2].q = deg2rad(120);
	link[JY2].q = deg2rad(0);
	link[JP3].q = deg2rad(-145);
	link[JY3].q = deg2rad(0);
}

void Now_State( Link *link, int mode, int foot )
{
	int EDF = JY3;

	cout << endl << "IK_mode :";
	if(	mode == 0) cout << "NR-method" << endl;
	else if(mode ==1) cout << "LM-method" << endl;
	cout << endl << link[EDF].joint_name << "pos = [" << link[EDF].p(0) << ", " << link[EDF].p(1) << ", "<< link[EDF].p(2) << "]" << endl;
	cout << endl << link[EDF].joint_name << "Rot = [" << rad2deg(link[EDF].R(0)) << ", " << rad2deg(link[EDF].R(1)) << ", "<< rad2deg(link[EDF].R(2)) << "]" << endl;
	cout << endl << "J1 :" << rad2deg(link[JY1].q) << " deg" << endl;
					cout << "J2 :" << rad2deg(link[JP1].q) << " deg" << endl;
					cout << "J3 :" << rad2deg(link[JP2].q) << " deg" << endl;
					cout << "J4 :" << rad2deg(link[JY2].q) << " deg" << endl;
					cout << "J5 :" << rad2deg(link[JP3].q) << " deg" << endl;
					cout << "J6 :" << rad2deg(link[JY3].q) << " deg" << endl;
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
	printf("------------\n");
	printf("NR-IK: 'c'  \n");
	printf("LM-IK: 'n'  \n");
	printf("Reset : 'x' \n");
	printf("------------\n\n\n\n\n\n\n");
}

void State( Link *link, int mode, int foot )
{
	Now_State( link, mode, foot );
	print_usage();
}

void SetPlotConf(FILE *gp, int roll, int yaw )
{
	fprintf(gp, "set terminal qt size 800,850 font \"Arial,8\" title 'demo_biped'\n");
	fprintf(gp, "set xrange [-0.4:0.4]\n");
	fprintf(gp, "set yrange [-0.4:0.4]\n");
	fprintf(gp, "set zrange [ 0.0:0.4]\n");
	fprintf(gp, "set ticslevel 0\n");
	fprintf(gp, "set xtics 0.05\n");
	fprintf(gp, "set ytics 0.05\n");
	fprintf(gp, "set ztics 0.05\n");
	fprintf(gp, "set xlabel \"X(m)\"\n");
	fprintf(gp, "set ylabel \"Y(m)\"\n");
	fprintf(gp, "set zlabel \"Z(m)\"\n");
	fprintf(gp, "set grid\n");
	fprintf(gp, "set view \"%d\",\"%d\", 1, 1.2 \n",roll,yaw );
}

double _elm( Link* link, int root, int elem )
{
	return link[root].p(elem);
}

void PlotLeg( FILE *gp, Link* link, double c_x, double c_y, double c_z, double theta )
{
	int TOTAL = LINK_NUM;
	double x[TOTAL], y[TOTAL], z[TOTAL];

	int BCC = CC;
	int LS = JY1;
	int LG = JH;

	int i = 0;

	x[BCC] = _elm(link,BCC,0) + c_x;
	y[BCC] = _elm(link,BCC,1) + c_y;
	z[BCC] = _elm(link,BCC,2);

	for( i = LS; i <= LG; i++ )
	{
    x[i] = _elm(link,i,0) + c_x;
    y[i] = _elm(link,i,1) + c_y;
    z[i] = _elm(link,i,2);
	}

	fprintf(gp, "splot '-' with lines linetype 7 linewidth 10 title \"BASE\",\
					'-' with lines linetype 6 linewidth 10 title \"ROBOT\",\
					'-' with lines linetype 2 linewidth 10 title \"HAND\",\n");

	for( i = BCC; i <=LS; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");

	for( i = LS; i <= BCC+5; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");

	for( i = BCC+5; i <= BCC+7; i++)
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}
	fprintf(gp, "e\n");
	fflush(gp);
}
