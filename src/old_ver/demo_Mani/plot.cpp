#include "plot.h"

/* plot by gnuplot */

void SetPlotConf(FILE *gp, int roll, int yaw )
{
	fprintf(gp, "set terminal wxt size 800,800 font \"Arial,8\" title 'demo_Mani'\n");
	fprintf(gp, "set xrange [-0.4:0.4]\n");
	fprintf(gp, "set yrange [-0.4:0.4]\n");
	fprintf(gp, "set zrange [-0.4:0.4]\n");
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
	int TOTAL = JOINT_NUM + 5;
	double x[TOTAL], y[TOTAL], z[TOTAL];
	double S = sin( theta );
	double C = cos( theta );
	
	int BCC = BASE;
	int RFS = ARM0;
	int LFS = 8;
	int RFG = ARM5;
	int LFG = 9;

	int i = 0;

/* ===RF=== */

	x[BCC] = _elm(link,BCC,0) + c_x;
	y[BCC] = _elm(link,BCC,1) + c_y;
	z[BCC] = _elm(link,BCC,2);

	for( i = RFS; i <= RFG; i++ )
	{
    x[i] = _elm(link,i,0) + c_x;
    y[i] = _elm(link,i,1) + c_y;
    z[i] = _elm(link,i,2);
	}
#if 0	
	x[RFG+1] = _elm(link,RFG,0) + 0.05;
	y[RFG+1] = _elm(link,RFG,1) + c_y;
	z[RFG+1] = _elm(link,RFG,2);

/* ===LF=== */

	x[RFG+2] = _elm(link,BCC,0) + c_x;
	y[RFG+2] = _elm(link,BCC,1) + c_y;
	z[RFG+2] = _elm(link,BCC,2);

	for( i = LFS; i<= LFG; i++ )
	{
		x[i+2] = _elm(link,i,0) + c_x;
		y[i+2] = _elm(link,i,1) + c_y;
		z[i+2] = _elm(link,i,2);
	}

	x[LFG+3] = _elm(link,LFG,0) + 0.05;
	y[LFG+3] = _elm(link,LFG,1) + c_y;
	z[LFG+3] = _elm(link,LFG,2);


/* ==CC=== */

	x[LFG+4] = _elm(link,BCC,0) + c_x;
	y[LFG+4] = _elm(link,BCC,1) + c_y;
	z[LFG+4] = _elm(link,BCC,2);

	x[LFG+5] = _elm(link,BCC,0) + c_x;
  y[LFG+5] = _elm(link,BCC,1) + c_y;
  z[LFG+5] = _elm(link,BCC,2) + 0.02;
#endif
	fprintf(gp, "splot '-' with lines linetype 1 linewidth 5 title \"ARM\",\n");
/*
					'-' with lines linetype 3 linewidth 5 title \"LLEG\",\
					'-' with lines linetype 2 linewidth 5 title \"WAIST\",\n");
*/
	for( i = BCC; i < RFG; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");
/*
	for( i = LFS+1; i <= LFG+3; i++ )
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}

	fprintf(gp, "e\n");

	for( i = LFG + 4; i < TOTAL; i++)
	{
		fprintf( gp, "%f\t%f\t%f\n", x[i],y[i],z[i]);
	}
	fprintf(gp, "e\n");
*/
	fflush(gp);
}

