#include "MainControl.h"

double rad2deg(double radian){
    return radian * 180/M_PI;
}

double deg2rad(int deg){
    return deg * M_PI / 180;
}

MainControl::MainControl(int argc, char *argv[], const double dt)
{
  position_list_clear();
	kinematics = new Kinematics(ulink);
}

MainControl::~MainControl()
{
}

void MainControl::thread_run()
{
  update_position();
}

void MainControl::update_position()
{
  int cmd = -2, mode = 0, foot = 1;
  int target_link = 0;
	double x = 0.0, y = 0.0, z = 0.0;

	double lx, ly, lz, l_tr, l_tp, l_ty;

	double roll  = 0.0, pitch = 0.0, yaw   = 0.0;
	double bx = 0.0, by = 0.0, bz = 0.0, broll = 0.0, bpitch = 0.0, byaw = 0.0;
  double pos_step = 0.005;
  double rot_step = 1;

  system( "stty -echo");
  gp = popen("gnuplot", "w");
  PlotConf(74, 54);
  SetJointInfo(ulink);
  SetDefaultAngle();
  kinematics->calcForwardKinematics(CC);

  target_link = JY3;

  Link Target = ulink[target_link];
  bx = Target.p(0);
  by = Target.p(1);
  bz = Target.p(2);

  State();

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
			}

			else if(cmd == -1) break;
			else if(cmd == -2)
			{
				SetDefaultAngle();
				kinematics->calcForwardKinematics(CC);
				x = y = z = lx = ly = lz = 0.0;
				roll = pitch = yaw = l_tr = l_tp = l_ty = 0.0;
				Target = ulink[JY3];
			}
			State();
		}
		usleep(10000);

		Target.p << bx + x, by +y, bz + z;
		Target.R = kinematics->computeMatrixFromAngles( deg2rad(broll + roll), deg2rad(bpitch + pitch), deg2rad(byaw + yaw) );

		if(mode == 0)kinematics->calcInverseKinematics(target_link,Target);
		else if(mode ==1)kinematics->calcLMInverseKinematics(target_link,Target);
		PlotArm();
	}
	system( "stty echo");
  pclose(gp);
}

void MainControl::SetDefaultAngle()
{

  ulink[CC].q  = 0.0;
  ulink[JY1].q = deg2rad(0);
  ulink[JP1].q = deg2rad(25);
  ulink[JP2].q = deg2rad(120);
  ulink[JY2].q = deg2rad(0);
  ulink[JP3].q = deg2rad(-145);
  ulink[JY3].q = deg2rad(0);

}
void MainControl::Now_State()
{
  int EDF = JY3;

	cout << endl << "IK_mode :";
	if(	mode == 0) cout << "NR-method" << endl;
	else if(mode ==1) cout << "LM-method" << endl;
	cout << endl << ulink[EDF].joint_name << "pos = [" << ulink[EDF].p(0) << ", " << ulink[EDF].p(1) << ", "<< ulink[EDF].p(2) << "]" << endl;
	cout << endl << ulink[EDF].joint_name << "Rot = [" << rad2deg(ulink[EDF].R(0)) << ", " << rad2deg(ulink[EDF].R(1)) << ", "<< rad2deg(ulink[EDF].R(2)) << "]" << endl;
	cout << endl << "J1 :" << rad2deg(ulink[JY1].q) << " deg" << endl;
					cout << "J2 :" << rad2deg(ulink[JP1].q) << " deg" << endl;
					cout << "J3 :" << rad2deg(ulink[JP2].q) << " deg" << endl;
					cout << "J4 :" << rad2deg(ulink[JY2].q) << " deg" << endl;
					cout << "J5 :" << rad2deg(ulink[JP3].q) << " deg" << endl;
					cout << "J6 :" << rad2deg(ulink[JY3].q) << " deg" << endl;
}

void MainControl::print_usage()
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

void MainControl::State()
{
	Now_State();
	print_usage();
}

void MainControl::PlotConf(int roll, int yaw)
{
  fprintf(gp, "set terminal qt size 800,850 font \"Arial,8\" title 'demo_biped'\n");
  fprintf(gp, "set mouse\n");
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
  fprintf(gp, "set view \"%d\",\"%d\", 1, 1.2 \n",roll, yaw );
}

void MainControl::PlotArm()
{
  int TOTAL = LINK_NUM+1;
	double x[TOTAL], y[TOTAL], z[TOTAL];

	int BCC = CC;
	int LS = JY1;
	int LG = JH;

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);

	int i = 0;

	x[BCC] = ulink[BCC].p(0);
	y[BCC] = ulink[BCC].p(1);
	z[BCC] = ulink[BCC].p(2);

	for( i = LS; i <=LG; i++ )
	{
    x[i] = ulink[i].p(0);
    y[i] = ulink[i].p(1);
    z[i] = ulink[i].p(2);
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

	for( i = BCC+6; i <= LG; i++)//BCC+7
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}
	fprintf(gp, "e\n");

	fflush(gp);
}


/*
void MainControl::
{

}
*/
